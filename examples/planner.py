import numpy as np
import enum
import time
from longthin import *

# Some code glued together to make the car park itself from a random position
# TODO: Refactor this code to make it more readable and maintainable


class State(enum.Enum):
    IDLE = 0
    TO_CENTER = 1
    ORBIT = 2
    TO_SPOT = 3
    TO_SPOT2 = 4
    TO_SPOT3 = 5
    PARK = 6


def cb_lane_vision(packet):
    global mean_x, min_y
    mean_x = packet.mean_x
    min_y = packet.min_y


def cb_goal_vision(packet):
    global goal_x, goal_area
    goal_x = packet.center_x
    goal_area = packet.area


def cb_ekf(packet):
    global ekf_pos, ekf_yaw
    ekf_pos = np.array([packet.x, packet.y])
    ekf_yaw = packet.yaw


def cb_button(packet):
    global button_park, state
    index = packet.index
    pressed = packet.state
    if index == 0 and pressed:
        if state == State.IDLE:
            button_park = True
        else:
            packet = Motor(0, 0)
            node.publish(packet)
            state = State.IDLE


node = LTNode()
state = State.IDLE
button_park = False
ekf_pos = np.array([1e6, 1e6])
ekf_yaw = 0
goal_x = None
goal_area = None


controller = ParkingController(node)
parking_est = ParkingEstimator(node)
node.subscribe(LaneVision, cb_lane_vision)
node.subscribe(EkfState, cb_ekf)
node.subscribe(GoalVision, cb_goal_vision)
node.subscribe(ButtonState, cb_button)

while True:
    node.spin_once()

    print(state)
    if state == State.IDLE:
        if button_park:
            state = State.TO_CENTER
            node.publish(EkfReset(0))
            parking_est.unsubscribe()
            parking_est = ParkingEstimator(node)
            controller.disable()
            button_park = False
            goal_x = None
            goal_area = None
            mean_x = None
            min_y = None
    elif state == State.TO_CENTER:
        packet = SetpointPos(0, 0)
        node.publish(packet)
        if np.linalg.norm(ekf_pos) < 0.2:
            state = State.ORBIT
            heading = np.array([np.cos(ekf_yaw), np.sin(ekf_yaw)])
            direction = heading[0] * ekf_pos[1] - heading[1] * ekf_pos[0]
        t_start = time.time()
        if parking_est.measurement_count > 40:
            state = State.TO_SPOT2
    elif state == State.ORBIT:
        u = (time.time() - t_start) / 90 * 0.8 + 0.2
        if direction < 0:
            packet = Motor(u, 1.0)
        else:
            packet = Motor(1.0, u)
        node.publish(packet)
        if goal_x is not None and abs(goal_x) < 0.5:
            state = State.TO_SPOT
    elif state == State.TO_SPOT:
        controller.to_goal = True
        controller.enabled = True
        if parking_est.measurement_count > 40:
            state = State.TO_SPOT2
    elif state == State.TO_SPOT2:
        controller.to_goal = False
        controller.enabled = False
        packet = SetpointPos(*parking_est.approach_pos)
        node.publish(packet)
        if np.linalg.norm(ekf_pos - parking_est.approach_pos) < 0.2:
            state = State.TO_SPOT3
    elif state == State.TO_SPOT3:
        packet = SetpointPos(*parking_est.spot_pos)
        node.publish(packet)
        if (goal_area is not None
                and goal_area > 0.007
                and mean_x is not None
                and np.linalg.norm(parking_est.spot_pos - ekf_pos) < 1.0
            ):
            state = State.PARK
    elif state == State.PARK:
        controller.enabled = True
        controller.to_goal = False

        if not controller.is_parking:
            packet = Motor(0, 0)
            node.publish(packet)
            state = State.IDLE

    time.sleep(1e-2)
