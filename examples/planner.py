import numpy as np
import enum
import time
from longthin import *


class State(enum.Enum):
    IDLE = 0
    TO_CENTER = 1
    ORBIT = 2
    PINPOINT = 3
    ALIGNMENT = 4
    APPROACH = 5
    APPROACH_NOEV = 6
    PARK = 7


def cb_lane_vision(packet):
    global mean_x, min_y
    mean_x = packet.mean_x
    min_y = packet.min_y


def cb_goal_vision(packet):
    global goal_x, goal_area
    goal_x = packet.center_x
    goal_area = packet.area


def cb_button(packet):
    global button_park, state
    index = packet.index
    pressed = packet.state
    if index == 0 and pressed:
        if state == State.IDLE:
            button_park = True
        else:
            # Abort parking
            hcontroller.set_mode(ControllerMode.MANUAL)
            hcontroller.setpoint(0, 0)
            state = State.IDLE


def cb_ev_pose(packet):
    global last_ev_t
    last_ev_t = time.time()


node = LTNode()
state = State.IDLE
prev_state = State.IDLE
state_change = False
button_park = False
goal_x = None
goal_area = None
last_ev_t = 0


hcontroller = HighLevelController(node)
parking_est = ParkingEstimator(node)
node.subscribe(LaneVision, cb_lane_vision)
node.subscribe(GoalVision, cb_goal_vision)
node.subscribe(ButtonState, cb_button)
node.subscribe(EvPose, cb_ev_pose)

while True:
    node.spin_once()
    if state != prev_state:
        prev_state = state
        state_change = True
        print("State:", state)
    else:
        state_change = False

    if state == State.IDLE:
        if state_change:
            hcontroller.set_mode(ControllerMode.MANUAL)
            hcontroller.setpoint(0, 0)

        if button_park:
            node.publish(EkfReset(0))
            parking_est.unsubscribe()
            parking_est = ParkingEstimator(node)
            button_park = False
            goal_x = None
            goal_area = None
            mean_x = None
            min_y = None
            state = State.TO_CENTER

    elif state == State.TO_CENTER:
        if state_change:
            hcontroller.set_mode(ControllerMode.DUBINS)
            hcontroller.setpoint(0, 0, circle=True)

        if hcontroller.is_reached:
            # TODO: A hack to get the direction of the orbit
            direction = hcontroller.controller.gen_path[2].n[2]
            state = State.ORBIT

        if parking_est.measurement_count > 50:
            state = State.ALIGNMENT
    elif state == State.ORBIT:
        if state_change:
            t_start = time.time()
            hcontroller.set_mode(ControllerMode.MANUAL)

        u = (time.time() - t_start) / 90 * 0.8 + 0.2
        if direction < 0:
            hcontroller.setpoint(u, 1.0)
        else:
            hcontroller.setpoint(1.0, u)

        if goal_x is not None and abs(goal_x) < 0.5:
            state = State.PINPOINT
    elif state == State.PINPOINT:
        if state_change:
            hcontroller.set_mode(ControllerMode.GOAL)
            hcontroller.setpoint()

        if parking_est.measurement_count > 50:
            state = State.ALIGNMENT
    elif state == State.ALIGNMENT:
        if state_change:
            hcontroller.set_mode(ControllerMode.DUBINS)
            hcontroller.setpoint(*parking_est.approach_pos, circle=False)

        if hcontroller.is_reached:
            state = State.APPROACH
    elif state == State.APPROACH:
        if state_change:
            hcontroller.set_mode(ControllerMode.DUBINS)
            hcontroller.setpoint(*parking_est.spot_pos, circle=False)
            mean_x = None

        if time.time() - last_ev_t > 0.5:
            state = State.APPROACH_NOEV

        # TODO Parameterize this
        if (goal_area is not None
                    and goal_area > 0.005
                    and mean_x is not None
                ):
            state = State.PARK
    elif state == State.APPROACH_NOEV:
        if state_change:
            hcontroller.set_mode(ControllerMode.GOAL)
            hcontroller.setpoint()

        if (goal_area is not None
                    and goal_area > 0.005
                    and mean_x is not None
                ):
            state = State.PARK
    elif state == State.PARK:
        if state_change:
            hcontroller.set_mode(ControllerMode.PARKING)
            hcontroller.setpoint()

        if hcontroller.is_reached:
            state = State.IDLE

    packet = hcontroller.control()
    node.publish(packet)
    time.sleep(1e-2)
