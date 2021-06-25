import os
import math
import lgsvl
import cv2
from simple_pid import PID
import matplotlib.pyplot as plt


class SimState:
    time = 0.
    frame_id = 0
    speed = 0.
    wheel_ratio = 0.
    position = None
    velocity = None
    attitude = None




class SimControl:
    def __init__(self, logger, frame_dir, sim_map="SingleLaneRoad_highway", vehicle="Jaguar XE 2015 2001 sl_highway", sim_vel=[0.0, 0.0, 20.0], time_of_day=12.0):
        self.logger = logger
        self.sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
        # self.sim = lgsvl.Simulator("169.234.208.22", 8181)

        scene_name = sim_map
        if self.sim.current_scene == scene_name:
            self.sim.reset()
        else:
            self.sim.load(scene_name, seed=650387)  # fix the seed

        sim_speed = math.sqrt(sim_vel[0]**2 + sim_vel[1]**2 + sim_vel[2]**2)
        print(time_of_day)
        self.sim.set_time_of_day(float(time_of_day), fixed=True)
        print("Current time of day:", self.sim.time_of_day)

        spawns = self.sim.get_spawn()
        state = lgsvl.AgentState()
        state.transform = spawns[0]
        state.velocity = lgsvl.Vector(sim_vel[0], sim_vel[1], sim_vel[2])

        # sim_speed = 0
        # state.velocity = lgsvl.Vector(0, 0, 0)

        vehicle_name = vehicle

        self.ego = self.sim.add_agent(vehicle_name, lgsvl.AgentType.EGO, state)

        sensors = self.ego.get_sensors()
        self.main_cam = None
        self.free_cam = None
        for s in sensors:
            if s.name == "Main Camera":
                s.enabled = True
                self.main_cam = s
            elif s.name == "Free Camera":
                s.enabled = True
                self.free_cam = s
            else:
                s.enabled = False

        self.ego.on_collision(self.on_collision)

        # Car control knob, steering [-1, 1] +: right, -: left
        self.ctrl = lgsvl.VehicleControl()
        self.ctrl.throttle = 0.0
        self.ctrl.steering = 0.0
        self.ctrl.braking = 0.0
        self.ctrl.reverse = False
        self.ctrl.handbrake = False

        # 0=off, 1=low, 2=high beams
        if self.sim.time_of_day > 6.9 and self.sim.time_of_day < 18:
            self.ctrl.headlights = 0
        else:
            self.ctrl.headlights = 2

        self.ego.apply_control(self.ctrl, True)

        if False:
            # Spawn front NPC
            forward = lgsvl.utils.transform_to_forward(spawns[0])  # forward unit vector
            right = lgsvl.utils.transform_to_right(spawns[0])  # right unit vector
            state = lgsvl.AgentState()
            lateral_shift = 0
            # forward_shift = 58.0  # 2-sec safe distance: highway
            # forward_shift = 40.0  # 2-sec safe distance: local
            forward_shift = 87.0  # 3-sec safe distance: highway
            # forward_shift = 60.0  # 3-sec safe distance: local
            state.transform.position = spawns[0].position + lateral_shift * right + forward_shift * forward
            state.transform.rotation = spawns[0].rotation
            npc_front = self.sim.add_agent("Sedan", lgsvl.AgentType.NPC, state)
            npc_front.follow_closest_lane(True, 29)  # highway
            # npc_front.follow_closest_lane(True, 20)  # local


        if False:  # disabled for highway scenario
            # NPC spawn position (comment out if highway)
            forward = lgsvl.utils.transform_to_forward(spawns[0])  # forward unit vector
            right = lgsvl.utils.transform_to_right(spawns[0])  # right unit vector
            state = lgsvl.AgentState()
            lateral_shift = -2.7
            forward_shift = 274.0
            # forward_shift = 100.0
            state.transform.position = spawns[0].position + lateral_shift * right + forward_shift * forward
            state.transform.rotation = spawns[0].rotation
            state.transform.rotation.y = spawns[0].rotation.y + 180.0
            npc = self.sim.add_agent("BoxTruck", lgsvl.AgentType.NPC, state)
            #npc = self.sim.add_agent("Jeep", lgsvl.AgentType.NPC, state)
            # NPC follows lane
            npc.follow_closest_lane(True, 20)

        self.sim.run(1)

        self.curr_speed = self.ego.state.speed
        self.pid = PID(1.0, 0.1, 0.01, setpoint=sim_speed)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (-1, 1)

        ctrl_freq = 100
        self.ctrl_period = 1.0/ctrl_freq
        self.sim_time = 0.

        self.frame_id = 0
        self.frame_dir = frame_dir
        if not os.path.exists(self.frame_dir):
            self.logger.debug("Creating frame folder %s", self.frame_dir)
            os.mkdir(self.frame_dir)
        else:
            self.logger.debug("Frame folder already exist, overwriting %s", self.frame_dir)

    def on_collision(self, agent1, agent2, contact):
        name1 = "STATIC OBSTACLE" if agent1 is None else agent1.name
        name2 = "STATIC OBSTACLE" if agent2 is None else agent2.name
        self.logger.debug("XXXXXX: {} collided with {} at {}".format(name1, name2, contact))

    def next_yuv_frame(self):
        framePath = os.path.join(self.frame_dir, "frame_" + str(self.frame_id) + ".png")
        self.main_cam.save(framePath, compression=0)
        freeFramePath = os.path.join(self.frame_dir, "free_frame_" + str(self.frame_id) + ".png")
        # self.free_cam.save(freeFramePath, compression=0)
        bgr_img_raw = cv2.imread(framePath)
        bgr_img = cv2.resize(bgr_img_raw[200:-200,200:-200], (1164,874))
        # bgr_img = cv2.resize(bgr_img_raw[400:-400,400:-400], (1164,874))
        yuv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YUV_I420)
        ret_frame_id = self.frame_id
        self.frame_id += 1
        return ret_frame_id, yuv_img

    def apply_control(self, steering):
        if steering < -1 or steering > 1:
            self.logger.error("Steering is a ratio between [-1, 1], + means right")
        else:
            throttle = self.pid(self.curr_speed)
            self.logger.debug("Sim time %5.2f, speed %.2f, applying throttle %.2f",
                    self.sim_time, self.curr_speed, throttle)

            self.ctrl.throttle = throttle
            self.ctrl.steering = steering
            self.ego.apply_control(self.ctrl, True)  # sticky control
            self.sim.run(time_limit=self.ctrl_period)
            self.sim_time += self.ctrl_period
            self.curr_speed = self.ego.state.speed

        simState = self.get_sim_state()
        return simState

    def get_sim_state(self):
        simState = SimState()
        simState.time = self.sim_time
        simState.frame_id = self.frame_id
        simState.speed = self.ego.state.speed
        simState.wheel_ratio = self.ctrl.steering    # wheel angle ratio
        simState.position = [self.ego.state.position.x, self.ego.state.position.y, self.ego.state.position.z]
        simState.velocity = [self.ego.state.velocity.x, self.ego.state.velocity.y, self.ego.state.velocity.z]
        simState.attitude = [self.ego.state.rotation.x, self.ego.state.rotation.y, self.ego.state.rotation.z]
        return simState
