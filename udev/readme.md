Udev Rules
====================

This enables the computer to recognize specific USB devices and make them accessible on uniquely identifiable paths. This means that code that references these devices will always be able to find them on the vehicle.

# Usage

Run the following...

	~/ieee_ws/src/IEEE2017/udev/setup.sh

# Glossary

Hokuyo - A company that makes LIDARs (the laser sensors we use for SLAM)

udev - Linux rules you can set up so that a device doesn't automatically default to being at a port. A device will often automatically go to /dev/ttyUSB0, which is tough to work with if you have multiple devices and you serial interface requires a specific port to be chosen. This enables us to automatically put something it recognizes as a dynamixel at a port we make up, for example, /dev/dynamixel_tty
