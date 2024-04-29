import numpy as np
import enum
import time

from .ltpacket import *
from .controller import ControllerMode, HighLevelController
from .parking_estimator import ParkingEstimator


class State(enum.Enum):
    IDLE = 0
    TO_CENTER = 1
    ORBIT = 2
    PINPOINT = 3
    ALIGNMENT = 4
    APPROACH = 5
    APPROACH_NOEV = 6
    PARK = 7
    RESET = 8


class Planner:
    def __init__(self, node):
        self.node = node
        self.params = node.params
        self.state = State.IDLE
        self.prev_state = State.IDLE
        self.state_change = False
        self.button_park = False
        self.goal_x = None
        self.goal_area = None
        self.mean_x = None
        self.min_y = None
        self.ev_t = 0
        self.lane_t = 0
        self.goal_t = 0
        self.hcontroller = HighLevelController(node)
        self.parking_est = ParkingEstimator(node)
        self.node.subscribe(LaneVision, self.cb_lane_vision)
        self.node.subscribe(GoalVision, self.cb_goal_vision)
        self.node.subscribe(ButtonState, self.cb_button)
        self.node.subscribe(EvPose, self.cb_ev_pose)
        self.node.subscribe(EkfState, self.cb_ekf_state)
        self.state_functions = {
            State.IDLE: self.state_idle,
            State.TO_CENTER: self.state_to_center,
            State.ORBIT: self.state_orbit,
            State.PINPOINT: self.state_pinpoint,
            State.ALIGNMENT: self.state_alignment,
            State.APPROACH: self.state_approach,
            State.APPROACH_NOEV: self.state_approach_noev,
            State.PARK: self.state_park,
            State.RESET: self.state_reset,
        }
        self.t_start = 0  # Initialize a timing variable for ORBIT state

    def cb_lane_vision(self, packet):
        self.lane_t = time.time()
        self.mean_x = packet.mean_x
        self.min_y = packet.min_y
        self.step()

    def cb_goal_vision(self, packet):
        self.goal_t = time.time()
        self.goal_x = packet.center_x
        self.goal_area = packet.area
        self.step()

    def cb_button(self, packet):
        index = packet.index
        pressed = packet.state
        if index == 0 and pressed:
            if self.state == State.IDLE:
                self.button_park = True
            else:
                # Abort parking
                self.hcontroller.set_mode(ControllerMode.MANUAL)
                self.hcontroller.setpoint(0, 0)
                self.state = State.IDLE
        self.step()

    def cb_ev_pose(self, packet):
        self.ev_t = time.time()
        self.step()

    def cb_ekf_state(self, packet):
        self.step()

    def ev_is_recent(self):
        return time.time() - self.ev_t < 0.5

    def lane_is_recent(self):
        return self.mean_x is not None and time.time() - self.lane_t < 0.5

    def goal_is_recent(self):
        return self.goal_x is not None and time.time() - self.goal_t < 0.5

    def is_aligned(self):
        if self.goal_is_recent() and self.lane_is_recent():
            return abs(self.goal_x - self.mean_x) < 0.33
        return False

    def step(self):
        if self.state != self.prev_state:
            self.prev_state = self.state
            self.state_change = True
            print("State:", self.state)
        else:
            self.state_change = False

        self.state_functions[self.state]()
        if self.state != State.IDLE:
            packet = self.control()
            self.node.publish(packet)

    def control(self):
        packet = self.hcontroller.control()
        return packet

    def state_idle(self):
        if self.button_park:
            self.node.publish(EkfReset(0))
            self.parking_est.unsubscribe()
            self.parking_est = ParkingEstimator(self.node)
            self.button_park = False

            if self.is_aligned():
                self.state = State.PARK
                return

            self.state = State.TO_CENTER

    def state_to_center(self):
        if self.state_change:
            self.hcontroller.set_mode(ControllerMode.DUBINS)
            self.hcontroller.setpoint(0, 0, circle=True)

        if self.hcontroller.is_reached:
            # A hack to get the direction of the orbit
            self.direction = self.hcontroller.controller.gen_path[2].n[2]
            self.state = State.ORBIT

        if self.parking_est.measurement_count > self.params.planner_measurement_count:
            self.state = State.ALIGNMENT

    def state_orbit(self):
        if self.state_change:
            self.t_start = time.time()
            self.hcontroller.set_mode(ControllerMode.MANUAL)
            self.goal_x = None

        u = (time.time() - self.t_start) / 90 * 0.8 + 0.2
        if self.direction < 0:
            self.hcontroller.setpoint(u, 1.0)
        else:
            self.hcontroller.setpoint(1.0, u)

        if self.goal_is_recent() and abs(self.goal_x) < 0.5:
            self.state = State.PINPOINT

    def state_pinpoint(self):
        if self.state_change:
            self.hcontroller.set_mode(ControllerMode.GOAL)
            self.hcontroller.setpoint()

        if self.is_aligned():
            self.state = State.PARK
            return

        if self.parking_est.measurement_count > self.params.planner_measurement_count:
            self.state = State.ALIGNMENT

    def state_alignment(self):
        if self.state_change:
            self.hcontroller.set_mode(ControllerMode.DUBINS)
            self.hcontroller.setpoint(*self.parking_est.approach_pos, circle=False)

        if self.hcontroller.is_reached:
            self.state = State.APPROACH

    def state_approach(self):
        if self.state_change:
            self.hcontroller.set_mode(ControllerMode.DUBINS)
            self.hcontroller.setpoint(*self.parking_est.spot_pos, circle=False)
            self.mean_x = None
            self.goal_x = None

        if (self.goal_is_recent()
            and self.lane_is_recent()
                and self.goal_area > self.params.planner_goal_area_threshold):
            self.state = State.PARK

        if not self.ev_is_recent() and self.goal_is_recent():
            self.state = State.APPROACH_NOEV

    def state_approach_noev(self):
        if self.state_change:
            self.hcontroller.set_mode(ControllerMode.GOAL)
            self.hcontroller.setpoint()

        if (self.goal_is_recent()
            and self.lane_is_recent()
                and self.goal_area > self.params.planner_goal_area_threshold):
            self.state = State.PARK

    def state_park(self):
        if self.state_change:
            self.hcontroller.set_mode(ControllerMode.PARKING)
            self.hcontroller.setpoint()

        if self.hcontroller.is_reached:
            self.state = State.IDLE

    def state_reset(self):
        if self.state_change:
            self.hcontroller.set_mode(ControllerMode.MANUAL)
            self.hcontroller.setpoint(0, 0)
            self.goal_x = None
            self.goal_area = None
            self.mean_x = None
            self.min_y = None
            self.ev_t = 0
            self.lane_t = 0
            self.goal_t = 0
            self.reset_t = time.time()

        if time.time() - self.reset_t > 1:
            self.state = State.IDLE
