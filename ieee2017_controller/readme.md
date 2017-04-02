Vehicle Controllers
===================

Nodes that mid and high-level control for the vehicle. If you're curious about implementation, they are well-commented. For questions, check the git-blame for who to ask.


# Mecanum Controller
Given a desired linear and angular velocity for Shia, compute the individual wheel speeds to achieve those, while minimizing the sum of the squares of the wheel velocities.

The vehicle is overactuated (4 independent actuators on 3 degrees of freedom: yaw, x-translation and y-translation). As such, you have four equations in three variables, for converting a desired Twist to wheel actions. We solve this with linear least squares.


## Usage
    rosrun ieee2016_controller mecanum_controller.py
Publish to ```/twist```
With message type ```TwistStamped```
