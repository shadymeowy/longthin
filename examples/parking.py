from longthin import *

# This code uses the parking controller and the lane vision to park the car
# It uses a simple proportional controller to align the car with the parking spot
# The controller and lane vision can be tuned by related parameters

# Create the controller and node
node = LTNode()
controller = ParkingController(node.params)


# Callback function for the lane vision
def cb_lane_vision(packet):
    mean_x = packet.mean_x
    min_y = packet.min_y
    # Update the controller and get the control signal
    controller.update(mean_x, min_y)
    left, right = controller.control()
    # Publish the control signal as a motor packet
    packet = Motor(left, right)
    node.publish(packet)

# Subscribe to the lane vision
node.subscribe(LaneVision, cb_lane_vision)
# Run the node
node.spin()
