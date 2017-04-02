Navigation
=========

This package handles all of the navigation and localization nodes for the vehicle.

#Particle Filter
A [particle filter](https://en.wikipedia.org/wiki/Particle_filter) is a way of estimating our current location using sensor data. The reason we use a particle filter rather than SLAM is because the LiDAR data is not accurate due to the dark walls of the field. The particle filter will account for this by estimating the *best* position (not nessicarily the 100% correct position). Currently, the particle filter takes in odometry data and publishes an esitmated pose as well as a `map -> base_link` transform.

###LiDAR
The particle filter relies on LiDAR data in order to localize. To activate the LiDAR, run ```roslaunch ieee2017_navigation lidar.launch```. This will also launch the LiDAR Fuser script. This script takes the data from each of the LiDARs and combines it into one large scan. This combined scan is then passed to the particle filter for processing. Running this script assumes the Hokuyo Udev rules are set up (see the [udev folder](https://github.com/ufieeehw/IEEE2017/tree/master/udev)). Sometimes the LiDARs wont start, so you'll need to unplug and plug them in (someone should figure out why this is).

###GPGPU
This implementation of the particle fiter uses pyopencl for [General Purpose computing on Graphics Processing Units ](https://en.wikipedia.org/wiki/General-purpose_computing_on_graphics_processing_units), in other words the filter does most of its processing on the graphics processor rather then on the CPU. This allows for much more effiencent execution of the particle filter. For more information on how this works or how to get it installed, see Matthew Langford.

#EKF
We use an [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) to integrate odometry data from the wheel encoders and IMU data to generate a pose esitmation that is fed into the particle filter. We are using the EKF provided in the [robot_localization](http://wiki.ros.org/robot_localization) ros package.
