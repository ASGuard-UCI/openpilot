#!/usr/bin/env python3
import os
import sys
import gc
import time
import capnp
from cereal import car, log
from common.numpy_fast import clip, interp
from common.realtime import set_realtime_priority
import selfdrive.messaging as messaging
from selfdrive.config import Conversions as CV
# from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.controls.lib.vehicle_model import VehicleModel

from selfdrive.car.car_helpers import interfaces

from selfdrive.controls.simcontrol import SimControl, SimState
import logging


def init_logger():
  logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s')
  logger = logging.getLogger("lgsvlsim")
  logger.setLevel(logging.DEBUG)
  return logger


State = log.ControlsState.OpenpilotState

##############################################
# Toyota parameters
ANGLE_MAX_BP = [0., 5.]
ANGLE_MAX_V = [510., 300.]
ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .15]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.4]

# # Tesla parameters
# ANGLE_MAX_BP = [0., 27., 36.]
# ANGLE_MAX_V = [410., 92., 36.]
# ANGLE_DELTA_BP = [0., 5., 15.]
# ANGLE_DELTA_V = [5., .8, .25]
# ANGLE_DELTA_VU = [5., 3.5, 0.8]
##############################################


def isActive(state):
  """Check if the actuators are enabled"""
  return state in [State.enabled, State.softDisabling]


def isEnabled(state):
  """Check if openpilot is engaged"""
  return (isActive(state) or state == State.preEnabled)


def update_state(CI, CC, curr_speed, curr_steer_angle):
  CS = CI.update(CC, None, curr_speed, curr_steer_angle)  # Junjie: update curr_speed in mock interface
  return CS


def data_send_no_can(sm, pm, CS, CI, CP, VM, state, v_cruise_kph):
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
    "vCruise": float(v_cruise_kph),
  }
  pm.send('controlsState', dat)

  # carState
  cs_send = messaging.new_message()
  cs_send.init('carState')
  cs_send.valid = CS.canValid
  cs_send.carState = CS
  pm.send('carState', cs_send)

  return


def control_thread(frame_dir, sim_map, vehicle, sim_vel, time_of_day):
  gc.disable()
  set_realtime_priority(3)

  logger = init_logger()
  sim = SimControl(logger, frame_dir, sim_map, vehicle, sim_vel, time_of_day)

  pm = messaging.PubMaster(['controlsState', 'frame', 'simState', 'carState'])
  sm = messaging.SubMaster(['pathPlan'])

  # Junjie: get mock car parameters and interface instead
  CarInterface, CarController = interfaces['mock']
  CP = CarInterface.get_params('mock', None)
  CI = CarInterface(CP, CarController)

  CC = car.CarControl.new_message()

  VM = VehicleModel(CP)

  # state = State.disabled
  state = State.enabled
  v_cruise_kph = 255
  v_cruise_kph_last = 0

  sm['pathPlan'].sensorValid = True
  sm['pathPlan'].posenetValid = True
  curr_speed = 0.
  curr_steer_angle = 0.
  CS = update_state(CI, CC, curr_speed, curr_steer_angle)
  last_angle = CS.steeringAngle

  frame_id, yuv_img = sim.next_yuv_frame()
  time.sleep(1)
  frame_dat = messaging.new_message()
  frame_dat.init('frame')
  frame_dat.frame.frameId = frame_id
  frame_dat.frame.image = yuv_img.tobytes()
  pm.send('frame', frame_dat)
  logger.debug("Sim time %5.2f, frame %d sent", sim.sim_time, frame_id)

  time.sleep(1)

  while True:
    frame_id, yuv_img = sim.next_yuv_frame()
    frame_dat = messaging.new_message()
    frame_dat.init('frame')
    frame_dat.frame.frameId = frame_id
    frame_dat.frame.image = yuv_img.tobytes()
    pm.send('frame', frame_dat)
    logger.debug("Sim time %5.2f, frame %d sent", sim.sim_time, frame_id)

    sm.update()
    if sm.updated['pathPlan'] and sm['pathPlan'].mpcSolutionValid and sm['pathPlan'].sensorValid and sm['pathPlan'].paramsValid:
      path_plan = sm['pathPlan']
      logger.debug("Recv'd PathPlan: angle offset %s, desired steer angle %s", "{:.2f}".format(path_plan.angleOffset), "{:.4f}".format(path_plan.angleSteers))
      angle_steers_des = path_plan.angleSteers - path_plan.angleOffset  # desired steering angle
      angle_steers_cur = CS.steeringAngle - path_plan.angleOffset  # current steering angle

      for i in range(5):
        curr_speed = CS.vEgo
        apply_angle = angle_steers_des
        angle_lim = interp(curr_speed, ANGLE_MAX_BP, ANGLE_MAX_V)
        apply_angle = clip(apply_angle, -angle_lim, angle_lim)
        if last_angle * apply_angle > 0. and abs(apply_angle) > abs(last_angle):
          angle_rate_lim = interp(curr_speed, ANGLE_DELTA_BP, ANGLE_DELTA_V)
        else:
          angle_rate_lim = interp(curr_speed, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
        apply_angle = clip(apply_angle, last_angle - angle_rate_lim, last_angle + angle_rate_lim)

        # Junjie: LGSVL accepts a angle (not delta) in a range [-1, 1], +: right
        apply_angle_ratio = -apply_angle/ANGLE_MAX_V[0]

        logger.debug("speed %s, last steer angle %s, applying steer angle %s", "{:.2f}".format(curr_speed), "{:.2f}".format(last_angle), "{:.2f}".format(apply_angle))
        last_angle = apply_angle

        sim_state = sim.apply_control(apply_angle_ratio)
        # sim_state = sim.apply_control(-9.0/300)

        curr_speed = sim_state.speed
        curr_steer_angle = -sim_state.wheel_ratio * ANGLE_MAX_V[0]

        sim_dat = messaging.new_message()
        sim_dat.init('simState')
        sim_dat.simState.time = sim_state.time
        sim_dat.simState.frameId = sim_state.frame_id
        sim_dat.simState.speed = sim_state.speed
        sim_dat.simState.steer = curr_steer_angle
        sim_dat.simState.desiredSteer = path_plan.angleSteers
        sim_dat.simState.position = sim_state.position
        sim_dat.simState.velocity = sim_state.velocity
        sim_dat.simState.attitude = sim_state.attitude
        pm.send('simState', sim_dat)


        CS = update_state(CI, CC, curr_speed, curr_steer_angle)
        data_send_no_can(sm, pm, CS, CI, CP, VM, state, v_cruise_kph)


def main(frame_dir, sim_map, vehicle, sim_vel_x, sim_vel_y, sim_vel_z, time_of_day):
  control_thread(frame_dir, sim_map, vehicle, [sim_vel_x, sim_vel_y, sim_vel_z], time_of_day)


if __name__ == "__main__":
  main()
