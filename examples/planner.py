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
    global goal_x, goal_area, approach_x, approach_y, spot_x, spot_y
    goal_x = packet.center_x
    goal_area = packet.area

    fx = config.camera.model.fx
    width = config.camera.model.width
    mean_x = goal_x * (0.5 * width)
    angle = np.arctan(mean_x / fx)
    yaw = np.deg2rad(ekf_yaw)
    angle = yaw + angle

    x = ekf_pos[0]
    y = ekf_pos[1]

    xp = None
    yp = None

    y0 = area_h / 2 + distance
    x0 = x + (area_h/2 - y + distance) / np.tan(angle)
    angle0 = (np.arctan2(y0 - y, x0 - x) - yaw) % (2 * np.pi)
    if (angle0 < np.pi / 2 or angle0 > 3 * np.pi / 2) and (-area_w < x0 < area_w):
        xp = x0
        yp = y0 - distance
        entry_xs0.append(xp)
        entry_ys0.append(yp)

    y1 = -area_h / 2 - distance
    x1 = x + (-area_h/2 - y - distance) / np.tan(angle)
    angle1 = (np.arctan2(y1 - y, x1 - x) - yaw) % (2 * np.pi)
    if (angle1 < np.pi / 2 or angle1 > 3 * np.pi / 2) and (-area_w < x1 < area_w):
        xp = x1
        yp = y1 + distance
        entry_xs1.append(xp)
        entry_ys1.append(yp)

    x2 = area_w / 2 + distance
    y2 = y + (area_w/2 - x + distance) * np.tan(angle)
    angle2 = (np.arctan2(y2 - y, x2 - x) - yaw) % (2 * np.pi)
    if (angle2 < np.pi / 2 or angle2 > 3 * np.pi / 2) and (-area_h < y2 < area_h):
        xp = x2 - distance
        yp = y2
        entry_xs2.append(xp)
        entry_ys2.append(yp)

    x3 = -area_w / 2 - distance
    y3 = y + (-area_w/2 - x - distance) * np.tan(angle)
    angle3 = (np.arctan2(y3 - y, x3 - x) - yaw) % (2 * np.pi)
    if (angle3 < np.pi / 2 or angle3 > 3 * np.pi / 2) and (-area_h < y3 < area_h):
        xp = x3 + distance
        yp = y3
        entry_xs3.append(xp)
        entry_ys3.append(yp)

    mx = np.max([len(entry_xs0), len(entry_xs1), len(entry_xs2), len(entry_xs3)])
    if len(entry_xs0) == mx:
        spot_x = np.mean(entry_xs0)
        spot_y = np.mean(entry_ys0)
        spot_x = max(-1., min(1., spot_x))
        approach_x = spot_x
        approach_y = spot_y - 2.0
    elif len(entry_xs1) == mx:
        spot_x = np.mean(entry_xs1)
        spot_y = np.mean(entry_ys1)
        spot_x = max(-1., min(1., spot_x))
        approach_x = spot_x
        approach_y = spot_y + 2.0
    elif len(entry_xs2) == mx:
        spot_x = np.mean(entry_xs2)
        spot_y = np.mean(entry_ys2)
        spot_y = max(-1., min(1., spot_y))
        approach_x = spot_x - 2.0
        approach_y = spot_y
    else:
        spot_x = np.mean(entry_xs3)
        spot_y = np.mean(entry_ys3)
        spot_y = max(-1., min(1., spot_y))
        approach_x = spot_x + 2.0
        approach_y = spot_y


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
config = node.config
state = State.IDLE
button_park = False
ekf_pos = np.array([0, 0])
ekf_yaw = 0
goal_x = None
goal_area = None
mean_x = None
min_y = None
entry_xs0 = []
entry_ys0 = []
entry_xs1 = []
entry_ys1 = []
entry_xs2 = []
entry_ys2 = []
entry_xs3 = []
entry_ys3 = []
area_w = config.renderer.area.width
area_h = config.renderer.area.height
distance = config.renderer.spot.length


controller = ParkingController()
node.subscribe(LaneVision, cb_lane_vision)
node.subscribe(GoalVision, cb_goal_vision)
node.subscribe(EkfState, cb_ekf)
node.subscribe(ButtonState, cb_button)

while True:
    node.spin_once()

    print(state)
    if state == State.IDLE:
        if button_park:
            state = State.TO_CENTER
            button_park = False
            node.publish(EkfReset(0))
            entry_xs0[:] = []
            entry_ys0[:] = []
            entry_xs1[:] = []
            entry_ys1[:] = []
            entry_xs2[:] = []
            entry_ys2[:] = []
            entry_xs3[:] = []
            entry_ys3[:] = []
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
        if len(entry_xs0) + len(entry_xs1) + len(entry_xs2) + len(entry_xs3) > 30:
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
        controller.active = True
        controller.update(goal_x, 0)
        left, right = controller.control()
        packet = Motor(left, right)
        node.publish(packet)
        if len(entry_xs0) + len(entry_xs1) + len(entry_xs2) + len(entry_xs3) > 60:
            state = State.TO_SPOT2
    elif state == State.TO_SPOT2:
        packet = SetpointPos(approach_x, approach_y)
        node.publish(packet)
        if np.linalg.norm(ekf_pos - np.array([approach_x, approach_y])) < 0.2:
            state = State.TO_SPOT3
    elif state == State.TO_SPOT3:
        packet = SetpointPos(spot_x, spot_y)
        node.publish(packet)
        if (goal_area is not None
            and goal_area > 0.007
            and mean_x is not None
            and np.linalg.norm([spot_x, spot_y] - ekf_pos) < 1.0
            ):
            state = State.PARK
    elif state == State.PARK:
        controller.update(mean_x, min_y)
        left, right = controller.control()
        packet = Motor(left, right)
        node.publish(packet)
        if not controller.active:
            packet = Motor(0, 0)
            node.publish(packet)
            state = State.IDLE

    time.sleep(0.1)
