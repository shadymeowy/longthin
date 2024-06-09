# Long Thin Hauler - Graduation Project

Welcome to the official GitHub repository for the "Long Thin Hauler," an innovative autonomous vehicle designed to navigate and self-park within a constrained  environment. This project is a result of extensive research and development by the BeyondTech team as part of our final-year project for METU Electrical Electronics Engineering program.

## Project Overview

The "Long Thin Hauler" is an autonomous ground vehicle optimized for precise parking within narrow spaces. Using a combination of sensor fusion, wireless communication, and visual-based algorithms, our vehicle can autonomously navigate and park in a designated area measuring 3m x 3m, no matter its starting position.

### Features
- **Autonomous Navigation**: Navigate autonomously using sensor data and localization techniques.
- **Wireless Communication**: Components communicate wirelessly in a plug-and-play manner using a custom but extensible communication protocol.
- **Visual Positioning**: Utilizes cameras and fiducial markers for precise positioning and navigation.
- **Custom Mechanical Design**: Features a unique mechanical design optimized for narrow spaces.

## Getting Started

To get started with the Long Thin Hauler project, clone this repository and install as a Python package.
```bash
git clone https://github.com/shadymeowy/longthin
cd longthin
pip install .
```

## Usage
While the Long Thin Hauler is designed for a specific application, the core components can be adapted for other projects. There are several modules available for use, including:

- `longthin.filter`: Includes an extended Kalman filter for sensor fusion and localization utilizing preintegration of IMU data with visual odometry. Generated symbolically using the `sympy` library.
- `longthin.controller`: Includes many controllers such as visual odometry based controller, Dubins path controller, and a PID on the heading controller.
- `longthin.calibration`: Includes IMU calibration for accelerometer and magnetometer using the ellipsoid fitting method.
- `longthin.geometry`: Includes geometry functions for calculating distances, angles, and transformations.
- `longthin.graphics`: Includes functions for rendering the vehicle and environment in a 3D simulation.
- `longthin.model`: A full vehicle model with dynamics and kinematics for simulation and control.
- `longthin.path`: Includes functions for generating and manipulating Dubins paths using pure vector algebra based on the Dubins path equations.
- `longthin.ui`: Qt6 based dockable UI for visualizing, tuning and monitoring the vehicle.

The project also includes a communication protocol for different transport layers and a simulation environment for testing and development.  While using this project on actual hardware not trivial. However, it is easy to use the modules in a simulation environment. The simulation environment is designed to be modular and flexible. Any module that is compatible with the simulation environment can be used to test and develop the algorithms.

To run the simulation environment, use the following command:
```bash
longthin-runner --config sim.yaml
```
This command will run necessary modules and start the simulation environment. This method requires the `tmux` terminal multiplexer to be installed. If you don't have `tmux` installed, you can run the modules separately. To run the modules separately, check `longthin/config/sim.yaml` for the necessary modules.

One of the most critical feature of the project is the communication protocol that allows the modules to communicate with each other. The messages can be defined once, then can be used on microcontrollers, simulation environments, or any other platform. The project supports the various transport layers such as ZeroMQ (with its associated transport layers), UART, and ESPNOW.
Any module (software or hardware) can communicate with each other in a plug-and-play manner. To see how the communication protocol works on the firmware side, check the `firmware` directory.

The firmware also includes a few AHRS implementations for sensor fusion. While the project itself implemented for rp2040, the individual parts are included in C99 and can be used on any platform. See the `firmware` directory for more information.

## Documentation
There is no dedicated documentation for the project. However, we believe the project structure is quite self-explanatory. The code is well-documented, and the modules are designed to be modular and reusable. If you want the read the associated report, you can find it in the `report` directory. I cannot guarantee the completeness of the report. While implementing a robust system is hard, documenting the methods and implementation details is quite impossible if there is no incentive to do so.

## Contributing
While this is a mere student project of ours, it may serve as a starting point for your own projects. We hope you find it useful and welcome any feedback or contributions. Feel free to reach out to us with any questions or suggestions.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.