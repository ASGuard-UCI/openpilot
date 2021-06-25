#!/usr/bin/env python3
import os
import sys
import gc
import capnp
from cereal import car, log
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot, set_realtime_priority, DT_CTRL
import selfdrive.messaging as messaging
from selfdrive.config import Conversions as CV
# from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.drive_helpers import get_events, \
                                                 create_event, \
                                                 EventTypes as ET
from selfdrive.controls.lib.longcontrol import LongControl
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.alertmanager import AlertManager
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.locationd.calibration_helpers import Calibration

from selfdrive.car.car_helpers import interfaces

import logging


def init_logger():
  logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s')
  logger = logging.getLogger("controlsd")
  logger.setLevel(logging.DEBUG)
  return logger


ThermalStatus = log.ThermalData.ThermalStatus
State = log.ControlsState.OpenpilotState

##############################################
# Toyota parameters
# ANGLE_MAX_BP = [0., 5.]
# ANGLE_MAX_V = [510., 300.]
# ANGLE_DELTA_BP = [0., 5., 15.]
# ANGLE_DELTA_V = [5., .8, .15]     # windup limit
# ANGLE_DELTA_VU = [5., 3.5, 0.4]

# Tesla parameters
ANGLE_MAX_BP = [0., 27., 36.]
ANGLE_MAX_V = [410., 92., 36.]
ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .25]
ANGLE_DELTA_VU = [5., 3.5, 0.8]
##############################################


def isActive(state):
  """Check if the actuators are enabled"""
  return state in [State.enabled, State.softDisabling]


def isEnabled(state):
  """Check if openpilot is engaged"""
  return (isActive(state) or state == State.preEnabled)


def data_sample(CS, sm, state):
  sm.update(0)
  events = list(CS.events)
  enabled = isEnabled(state)
  return events


def update_state(CI, CC, vEgo, steeringAngle):
  CS = CI.update(CC, None, vEgo, steeringAngle)  # Junjie: update vEgo in mock interface
  return CS


def data_send_no_can(sm, pm, CS, CI, CP, VM, state, events, v_cruise_kph, AM,
              LaC, LoC, start_time, events_prev):
  """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

  right_lane_visible = sm['pathPlan'].rProb > 0.5
  left_lane_visible = sm['pathPlan'].lProb > 0.5

  # controlsState
  dat = messaging.new_message()
  dat.init('controlsState')
  dat.valid = True
  dat.controlsState = {
    "pathPlanMonoTime": sm.logMonoTime['pathPlan'],
    "enabled": isEnabled(state),
    "active": isActive(state),
    "vEgo": CS.vEgo,
    "vEgoRaw": CS.vEgoRaw,
    "angleSteers": CS.steeringAngle,
    "curvature": VM.calc_curvature((CS.steeringAngle - sm['pathPlan'].angleOffset) * CV.DEG_TO_RAD, CS.vEgo),
    "steerOverride": CS.steeringPressed,
    "state": state,
    "engageable": not bool(get_events(events, [ET.NO_ENTRY])),
    "vCruise": float(v_cruise_kph),
  }
  pm.send('controlsState', dat)

  # carState
  cs_send = messaging.new_message()
  cs_send.init('carState')
  cs_send.valid = CS.canValid
  cs_send.carState = CS
  cs_send.carState.events = events
  pm.send('carState', cs_send)

  return


def controlsd_thread(sm=None, pm=None):
  logger = init_logger()
  gc.disable()

  # start the loop
  set_realtime_priority(3)

  # Pub/Sub Sockets
  if pm is None:
    pm = messaging.PubMaster(['controlsState', 'carState', 'opToSim'])

  if sm is None:
    sm = messaging.SubMaster(['pathPlan', 'gpsLocation', 'simState'], ignore_alive=['gpsLocation'])

  # Junjie: get mock car parameters and interface instead
  CarInterface, CarController = interfaces['mock']
  CP = CarInterface.get_params('mock', None)
  CI = CarInterface(CP, CarController)

  CC = car.CarControl.new_message()
  AM = AlertManager()

  LoC = LongControl(CP, CI.compute_gb)
  VM = VehicleModel(CP)

  if CP.lateralTuning.which() == 'pid':
    LaC = LatControlPID(CP)
  elif CP.lateralTuning.which() == 'indi':
    LaC = LatControlINDI(CP)
  elif CP.lateralTuning.which() == 'lqr':
    LaC = LatControlLQR(CP)

  # state = State.disabled
  state = State.enabled
  v_cruise_kph = 255
  v_cruise_kph_last = 0
  events_prev = []

  sm['pathPlan'].sensorValid = True
  sm['pathPlan'].posenetValid = True
  vEgo = 0.
  steeringAngle = 0.
  CS = update_state(CI, CC, vEgo, steeringAngle)
  last_angle = CS.steeringAngle

  while True:
    start_time = sec_since_boot()
    # Sample data and compute car events
    events = data_sample(CS, sm, state)

    if sm['pathPlan'].mpcSolutionValid and sm['pathPlan'].sensorValid and sm['pathPlan'].paramsValid:
      path_plan = sm['pathPlan']
      logger.debug("Recv'd PathPlan: angle offset %s, desired steer angle %s", "{:.2f}".format(path_plan.angleOffset), "{:.4f}".format(path_plan.angleSteers))
      angle_steers_des = path_plan.angleSteers - path_plan.angleOffset  # desired steering angle
      angle_steers_cur = CS.steeringAngle - path_plan.angleOffset  # current steering angle

      for i in range(5):
        vEgo = CS.vEgo
        apply_angle = angle_steers_des
        angle_lim = interp(vEgo, ANGLE_MAX_BP, ANGLE_MAX_V)
        apply_angle = clip(apply_angle, -angle_lim, angle_lim)
        if last_angle * apply_angle > 0. and abs(apply_angle) > abs(last_angle):
          angle_rate_lim = interp(vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
        else:
          angle_rate_lim = interp(vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
        apply_angle = clip(apply_angle, last_angle - angle_rate_lim, last_angle + angle_rate_lim)

        # Junjie: LGSVL accepts a angle (not delta) in a range [-1, 1], +: right
        apply_angle_ratio = -apply_angle/ANGLE_MAX_V[0]

        logger.debug("speed %s, last steer angle %s, applying steer angle %s", "{:.2f}".format(vEgo), "{:.2f}".format(last_angle), "{:.2f}".format(apply_angle))
        last_angle = apply_angle

        ctrl_event = messaging.new_message()
        ctrl_event.init('opToSim')
        ctrl_event.opToSim.steering = apply_angle_ratio
        pm.send('opToSim', ctrl_event)

        sm.update()
        if sm.updated['simState']:
          vEgo = sm['simState'].speed
          steeringAngle = -sm['simState'].steer * ANGLE_MAX_V[0]
          logger.debug("Recv'd SimState from LGSVL")
        elif sm.updated['pathPlan']:
          logger.debug("Failed to recv SimState, but a PathPlan")
        else:
          logger.debug("Failed to recv SimState")

        CS = update_state(CI, CC, vEgo, steeringAngle)

        data_send_no_can(sm, pm, CS, CI, CP, VM, state, events,
                v_cruise_kph, AM, LaC, LoC,
                start_time, events_prev)


def main(sm=None, pm=None, logcan=None):
  controlsd_thread(sm, pm)


if __name__ == "__main__":
  main()
