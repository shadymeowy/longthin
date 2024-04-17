from longthin import *

# This code uses the parking controller and the lane vision to park the car
# It uses a simple proportional controller to align the car with the parking spot
# The controller and lane vision can be tuned by related parameters

# Create the controller and node
node = LTNode()
controller = ParkingController(node)

# Enable the controller
controller.enable()

# Run the node
node.spin()
