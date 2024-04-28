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


pcontrol = ParkingController(node)
dcontrol = DubinsController(node)
parking_est = ParkingEstimator(node)
node.subscribe(LaneVision, cb_lane_vision)
node.subscribe(EkfState, cb_ekf)
node.subscribe(GoalVision, cb_goal_vision)
node.subscribe(ButtonState, cb_button)

while True:
    node.spin_once()

    print(state)
    if state == State.IDLE:
        pcontrol.enabled = False
        dcontrol.enabled = False
        packet = Motor(0, 0)
        node.publish(packet)
        if button_park:
            node.publish(EkfReset(0))
            parking_est.unsubscribe()
            parking_est = ParkingEstimator(node)
            button_park = False
            goal_x = None
            goal_area = None
            mean_x = None
            min_y = None

            pcontrol.enabled = False
            pcontrol.is_parking = True
            dcontrol.set_target(0, 0, circle=True)
            dcontrol.enabled = True
            state = State.TO_CENTER
    elif state == State.TO_CENTER:
        if dcontrol.target_reached:
            dcontrol.enabled = False
            direction = dcontrol.gen_path[2].n[2]
            t_start = time.time()
            state = State.ORBIT
        if parking_est.measurement_count > 50:
            dcontrol.set_target(*parking_est.approach_pos, circle=False)
            dcontrol.enabled = True
            state = State.TO_SPOT2
    elif state == State.ORBIT:
        u = (time.time() - t_start) / 90 * 0.8 + 0.2
        if direction < 0:
            packet = Motor(u, 1.0)
        else:
            packet = Motor(1.0, u)
        node.publish(packet)
        if goal_x is not None and abs(goal_x) < 0.5:
            pcontrol.to_goal = True
            pcontrol.enabled = True
            state = State.TO_SPOT
    elif state == State.TO_SPOT:
        if parking_est.measurement_count > 50:
            pcontrol.to_goal = False
            pcontrol.enabled = False
            dcontrol.set_target(*parking_est.approach_pos, circle=False)
            dcontrol.enabled = True
            state = State.TO_SPOT2
    elif state == State.TO_SPOT2:
        if dcontrol.target_reached:
            dcontrol.set_target(*parking_est.spot_pos, circle=False)
            dcontrol.enabled = True
            state = State.TO_SPOT3
    elif state == State.TO_SPOT3:
        # TODO Parameterize this
        if (goal_area is not None
                and goal_area > 0.007
                and mean_x is not None
            ):
            pcontrol.enabled = True
            pcontrol.to_goal = False
            dcontrol.enabled = False
            state = State.PARK
    elif state == State.PARK:
        if not pcontrol.is_parking:
            packet = Motor(0, 0)
            node.publish(packet)
            pcontrol.enabled = False
            dcontrol.enabled = False
            state = State.IDLE

    time.sleep(1e-2)
