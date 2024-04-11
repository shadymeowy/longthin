import numpy as np
from longthin import *

# This code uses a pure pursuit controller to follow a predefined circular path

# Define the path
radius = 0.8
origin = np.array([0, 0])
current_pos = np.array([0, 0])

# Define the controller parameters
lookahead = 0.5

# Create a controller
node = LTNode()
prev_angle = None


def cb_ekf(packet):
    global current_pos
    current_pos = np.array([packet.x, packet.y])
    target_pos = calculate_target(current_pos)
    packet = SetpointPos(target_pos[0], target_pos[1])
    node.publish(packet)


def calculate_target(current_pos):
    global prev_angle
    # Use pure pursuit to calculate the target point

    # Find the closest point on the path
    closest = current_pos - origin
    closest = closest / np.linalg.norm(closest)
    closest = origin + radius * closest

    # Find the angle of the closest point
    angle = np.arctan2(closest[1], closest[0])

    # Find the delta angle and target angle
    delta = lookahead / radius
    angle_target = angle + delta

    # Find the target point
    target = origin + radius * np.array([np.cos(angle_target), np.sin(angle_target)])
    return target

# Subscribe to the EKF state
node.subscribe(EkfState, cb_ekf)

# Run the node
node.spin()