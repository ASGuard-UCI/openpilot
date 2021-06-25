#!/usr/bin/env python3

import sys
import os
import time
import signal
import traceback
import numpy as np

from selfdrive.messaging.messaging_pyx import Context # pylint: disable=no-name-in-module, import-error
import selfdrive.messaging as messaging

import threading
from multiprocessing import Process
from setproctitle import setproctitle  #pylint: disable=no-name-in-module

from selfdrive.swaglog import cloudlog
import importlib
import configparser
import io

from common.basedir import BASEDIR
os.environ['BASEDIR'] = BASEDIR

sim_config = "simconfig.ini"
if not os.path.exists(sim_config):
    raise

config = configparser.ConfigParser(allow_no_value=True)
config.read(sim_config)
log_dir = config['log']['dir']
time_of_day = config['scenario']['time_of_day']
sim_map = config['scenario']['map']
vehicle = config['scenario']['vehicle']
sim_vel_x = config['scenario'].getfloat('velocity_x')
sim_vel_y = config['scenario'].getfloat('velocity_y')
sim_vel_z = config['scenario'].getfloat('velocity_z')

frame_dir = os.path.join(log_dir, 'frames')
poly_dir = os.path.join(log_dir, 'polys')
model_dir = os.path.join(log_dir, 'models')
trans_dir = os.path.join(log_dir, 'transforms')
state_dir = os.path.join(log_dir, 'states')

if not os.path.exists(log_dir):
    os.mkdir(log_dir)
if not os.path.exists(frame_dir):
    os.mkdir(frame_dir)
if not os.path.exists(poly_dir):
    os.mkdir(poly_dir)
if not os.path.exists(model_dir):
    os.mkdir(model_dir)
if not os.path.exists(trans_dir):
    os.mkdir(trans_dir)
if not os.path.exists(state_dir):
    os.mkdir(state_dir)

simargs = {"frame_dir": frame_dir, "sim_map": sim_map, "vehicle": vehicle, "sim_vel_x": sim_vel_x, "sim_vel_y": sim_vel_y, "sim_vel_z": sim_vel_z, "time_of_day": time_of_day}

persistent_processes = [
    "plannerd",
    "calibrationd",
    "visiond",
]

sim_processes = [
    "lgsvlsim",
]

managed_processes = {
    "thermald":     (True,   ["selfdrive.thermald"]),
    "uploader":     (True,   ["selfdrive.loggerd.uploader"]),
    "deleter":      (True,   ["selfdrive.loggerd.deleter"]),
    "controlsd":    (True,   ["selfdrive.controls.controlsd"]),
    "plannerd":     (True,   ["selfdrive.controls.plannerd"]),
    "radard":       (True,   ["selfdrive.controls.radard"]),
    "ubloxd":       (False,  ["selfdrive/locationd", ["./ubloxd"]]),
    "loggerd":      (False,  ["selfdrive/loggerd", ["./loggerd"]]),
    "logmessaged":  (True,   ["selfdrive.logmessaged"]),
    "tombstoned":   (True,   ["selfdrive.tombstoned"]),
    "logcatd":      (False,  ["selfdrive/logcatd", ["./logcatd"]]),
    "proclogd":     (False,  ["selfdrive/proclogd", ["./proclogd"]]),
    "boardd":       (False,  ["selfdrive/boardd", ["./boardd"]]),
    "pandad":       (True,   ["selfdrive.pandad"]),
    "ui":           (False,  ["selfdrive/ui", ["./start.py"]]),
    "calibrationd": (True,   ["selfdrive.locationd.calibrationd"]),
    "paramsd":      (False,  ["selfdrive/locationd", ["./paramsd"]]),
    "visiond":      (False,  ["selfdrive/visiond", ["./start.py"]]),
    "sensord":      (False,  ["selfdrive/sensord", ["./start_sensord.py"]]),
    "gpsd":         (False,  ["selfdrive/sensord", ["./start_gpsd.py"]]),
    "updated":      (True,   ["selfdrive.updated"]),
    "lgsvlsim":     (True,   ["selfdrive.controls.lgsvlsim"]),
}

running = {}

kill_processes = ['sensord', 'paramsd', 'visiond']


def kill_managed_process(name):
    if name not in running or name not in managed_processes:
        return
    cloudlog.info("killing %s" % name)

    if running[name].exitcode is None:
        if name in kill_processes:
            os.kill(running[name].pid, signal.SIGKILL)
        else:
            running[name].terminate()

        t = time.time()
        while time.time() - t < 5 and running[name].exitcode is None:
            time.sleep(0.001)

        if running[name].exitcode is None:
            cloudlog.info("killing %s with SIGKILL" % name)
            os.kill(running[name].pid, signal.SIGKILL)
            running[name].join()

    cloudlog.info("%s is dead with %d" % (name, running[name].exitcode))
    del running[name]


def cleanup_all_processes(signal, frame):
    cloudlog.info("caught ctrl-c %s %s" % (signal, frame))

    for name in list(running.keys()):
        kill_managed_process(name)
    cloudlog.info("everything is dead")


def launcher(proc):
  try:
    # import the process
    mod = importlib.import_module(proc)
    # rename the process
    setproctitle(proc)
    # create now context since we forked
    messaging.context = messaging.Context()
    # exec the process
    mod.main()
  except KeyboardInterrupt:
    cloudlog.warning("child %s got SIGINT" % proc)
  except Exception:
    # can't install the crash handler becuase sys.excepthook doesn't play nice
    # with threads, so catch it here.
    crash.capture_exception()
    raise


def simlauncher(proc):
  try:
    # import the process
    mod = importlib.import_module(proc)
    # rename the process
    setproctitle(proc)
    # create now context since we forked
    messaging.context = messaging.Context()
    # exec the process
    mod.main(**simargs)
  except KeyboardInterrupt:
    cloudlog.warning("child %s got SIGINT" % proc)
  except Exception:
    # can't install the crash handler becuase sys.excepthook doesn't play nice
    # with threads, so catch it here.
    crash.capture_exception()
    raise


def nativelauncher(pargs, cwd):
  # exec the process
  os.chdir(cwd)
  # because when extracted from pex zips permissions get lost -_-
  os.chmod(pargs[0], 0o700)
  os.execvp(pargs[0], pargs)


def start_managed_process(name):
    if name in running or name not in managed_processes:
        return
    proc = managed_processes[name]
    # if isinstance(proc, str):
    if proc[0]:
        cloudlog.info("starting python %s" % proc[1][0])
        if name == "lgsvlsim":
            running[name] = Process(name=name, target=simlauncher, args=(proc[1][0],))
        else:
            running[name] = Process(name=name, target=launcher, args=(proc[1][0],))
    else:
        pdir, pargs = proc[1]
        cwd = os.path.join(BASEDIR, pdir)
        cloudlog.info("starting process %s" % name)
        running[name] = Process(name=name, target=nativelauncher, args=(pargs, cwd))
    running[name].start()


def heartbeat():
    t = threading.Timer(5.0, heartbeat)
    t.daemon = True
    t.start()
    # check the status of all processes, did any of them die?
    running_list = ["   running %s %s" % (p, running[p]) for p in running]
    cloudlog.debug('\n'.join(running_list))


def manager_thread():
    sm = messaging.SubMaster(['simState', 'model'])

    # start persistent processes
    for p in persistent_processes:
        start_managed_process(p)

    cloudlog.info("wait for daemons to start...")
    time.sleep(4)
    cloudlog.info("start simulation")

    # start simulator and sending frames
    for p in sim_processes:
        start_managed_process(p)

    heartbeat()

    frame_id = 0
    while True:
        # simulation collection
        sm.update()
        if sm.updated['model']:
            polys = np.zeros((3,4))
            polys[0,:] = np.array(sm['model'].path.poly)
            polys[1,:] = np.array(sm['model'].leftLane.poly)
            polys[2,:] = np.array(sm['model'].rightLane.poly)

            trans = np.array(sm['model'].settings.inputTransform).reshape(3,3)
            np.save(poly_dir + '/poly_' + str(sm['model'].frameId), polys)
            np.save(trans_dir + '/trans_' + str(sm['model'].frameId), trans)

            # Junjie: newly added, to be extended later
            model = np.array([sm['model'].leftLane.prob, sm['model'].rightLane.prob])
            np.save(model_dir + '/model_' + str(sm['model'].frameId), model)

        if sm.updated['simState']:
            frame_id = sm['simState'].frameId
            sim_state = np.array(
                    [sm['simState'].time, sm['simState'].speed, sm['simState'].steer, sm['simState'].desiredSteer] +
                    list(sm['simState'].position) +
                    list(sm['simState'].velocity) +
                    list(sm['simState'].attitude))
            np.save(state_dir + '/state_' + str(sm['model'].frameId), sim_state)

        if frame_id > 250:
        #if frame_id > 150:
            # Terminate the simulation
            raise


def main():
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))
    try:
        manager_thread()
    except Exception:
        traceback.print_exc()
    finally:
        cloudlog.info("Clean up daemons..")
        cleanup_all_processes(None, None)


if __name__ == "__main__":
    main()
