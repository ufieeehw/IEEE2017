#!/bin/bash

NOCOLOR='\033[0m'
LOGCOLOR='\033[1;36m'
PASSCOLOR='\033[1;32m'
WARNCOLOR='\033[1;31m'

LOGPREFIX="${LOGCOLOR}INSTALLER:"
WARNPREFIX="${WARNCOLOR}ERROR:"
PASSTEXT="${PASSCOLOR}PASS"
FAILTEXT="${WARNCOLOR}FAIL"

instlog() {
	printf "$LOGPREFIX $@ $NOCOLOR\n"
}

instwarn() {
	printf "$WARNPREFIX $@ $NOCOLOR\n"
}


instpass() {
	printf "$PASSTEXT $NOCOLOR"
}


instfail() {
	printf "$FAILTEXT $NOCOLOR"
}

check_host() {

	# Attempts to ping a host to make sure it is reachable
	HOST="$1"

	HOST_PING=$(ping -c 2 $HOST 2>&1| grep "% packet" | cut -d" " -f 6 | tr -d "%")
	if ! [ -z "${HOST_PING}" ]; then

		# Uses packet loss percentage to determine if the connection is strong
		if [ $HOST_PING -lt 25 ]; then

			# Will return true if ping was successful and packet loss was below 25%
			return `true`
		else
			echo "There is a weak connection to the host"
		fi
	else
		echo "The server was unreachable"
	fi
	return `false`
}


#======================#
# Script Configuration #
#======================#

# Sane installation defaults for no argument cases
REQUIRED_OS="trusty"
CATKIN_DIR=~/ieee_ws

# Retrievs information about the location of the script
SCRIPT_PATH="`readlink -f ${BASH_SOURCE[0]}`"
SCRIPT_DIR="`dirname $SCRIPT_PATH`"

# Convert script arguments to variables
while [ "$#" -gt 0 ]; do
	case $1 in
		-h) printf "\nUsage: $0\n"
			printf "\n    [-c] catkin_workspace (Recommend: ~/ieee_ws)\n"
			printf "\n    example: ./install.sh -c ~/ieee_ws\n"
			printf "\n"
			exit 0
			;;
		-c) CATKIN_DIR="$2"
			shift 2
			;;
		-?) instwarn "Option $1 is not implemented"
			exit 1
			;;
	esac
done


#==================#
# Pre-Flight Check #
#==================#

instlog "Starting the pre-flight system check to ensure installation was done properly"

# The lsb-release package is critical to check the OS version
# It may not be on bare-bones systems, so it is installed here if necessary
sudo apt-get update -qq
sudo apt-get install -qq lsb-release

# Ensure that the correct OS is installed
DTETCTED_OS="`lsb_release -sc`"
if [ $DTETCTED_OS = $REQUIRED_OS ]; then
	OS_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	OS_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
echo "OS distribution and version check"

# Prevent the script from being run as root
if [ $USER != "root" ]; then
	ROOT_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	OS_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
echo "Running user check"

# Check whether or not github.com is reachable
# This also makes sure that the user is connected to the internet
if (check_host "github.com"); then
	NET_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	NET_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
echo "Internet connectivity check"

if !($OS_CHECK); then

	# The script will not allow the user to install on an unsupported OS
	instwarn "Terminating installation due to incorrect OS (detected $DTETCTED_OS)"
	instwarn "This project requires Ubuntu 14.04 (trusty)"
	exit 1
fi

if !($ROOT_CHECK); then

	# The script will not allow the user to install as root
	instwarn "Terminating installation due to forbidden user"
	instwarn "The install script should not be run as root"
	exit 1
fi

if !($NET_CHECK); then

	# The script will not allow the user to install without internet
	instwarn "Terminating installation due to the lack of an internet connection"
	instwarn "The install script needs to be able to connect to GitHub and other sites"
	exit 1
fi


#===================================================#
# Repository and Set Up and Main Stack Installation #
#===================================================#

# Make sure script dependencies are installed on bare bones installations
instlog "Installing install script dependencies"
sudo apt-get install -qq wget curl aptitude git

# Add software repositories for ROS and Gazebo
instlog "Adding ROS and Gazebo PPAs to software sources"
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu trusty main\" > /etc/apt/sources.list.d/ros-latest.list"
sudo sh -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main\" > /etc/apt/sources.list.d/gazebo-latest.list"

# Get the GPG signing keys for the above repositories
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

# Add software repository for Git-LFS
instlog "Adding the Git-LFS packagecloud repository to software sources"
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

# Install ROS and other project dependencies
instlog "Installing ROS Indigo base packages"
sudo apt-get update -qq
sudo apt-get install -qq python-catkin-pkg python-rosdep
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	sudo apt-get install -qq ros-indigo-desktop
else
	sudo apt-get install -qq ros-indigo-desktop-full
fi

# Break the ROS Indigo metapackage and install an updated version of Gazebo
instlog "Installing the latest version of Gazebo"
sudo aptitude unmarkauto -q '?reverse-depends(ros-indigo-desktop-full) | ?reverse-recommends(ros-indigo-desktop-full)'
sudo apt-get purge -qq ros-indigo-gazebo*
sudo apt-get install -qq gazebo7
sudo apt-get install -qq ros-indigo-gazebo7-msgs ros-indigo-gazebo7-ros ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros-control

# Source ROS configurations for bash on this user account
source /opt/ros/indigo/setup.bash
if !(cat ~/.bashrc | grep --quiet "source /opt/ros"); then
	echo "" >> ~/.bashrc
	echo "# Sets up the shell environment for ROS" >> ~/.bashrc
	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
fi

# Get information about ROS versions
instlog "Initializing ROS"
if !([ -f /etc/ros/rosdep/sources.list.d/20-default.list ]); then
	sudo rosdep init > /dev/null 2>&1
fi
rosdep update


#=================================#
# Workspace and Repository Set Up #
#=================================#

# Set up catkin workspace directory
if !([ -f $CATKIN_DIR/src/CMakeLists.txt ]); then
	instlog "Generating catkin workspace at $CATKIN_DIR"
	mkdir -p "$CATKIN_DIR/src"
	cd "$CATKIN_DIR/src"
	catkin_init_workspace
	catkin_make -C "$CATKIN_DIR"
else
	instlog "Using existing catkin workspace at $CATKIN_DIR"
fi

# Move the cloned git repository to the catkin workspace in semaphore
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	if [ -d ~/IEEE2017 ]; then
		mv ~/IEEE2017 "$CATKIN_DIR/src"
	fi
fi

# Source the workspace's configurations for bash on this user account
source "$CATKIN_DIR/devel/setup.bash"
if !(cat ~/.bashrc | grep --quiet "source $CATKIN_DIR/devel/setup.bash"); then
	echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc
fi

# Check if the IEEE2017 repository is present; if it isn't, download it
if !(ls "$CATKIN_DIR/src" | grep --quiet "IEEE2017"); then
	instlog "Downloading the IEEE2017 repository"
	cd $CATKIN_DIR/src
	git clone -q https://github.com/ufieeehw/IEEE2017.git
	cd $CATKIN_DIR/src/IEEE2017
	git remote rename origin upstream
	instlog "Make sure you change your git origin to point to your own fork! (git remote add origin your_forks_url)"
fi


#=========================#
# Dependency Installation #
#=========================#

instlog "Installing common dependencies from the Ubuntu repositories"

# Utilities for building and package management
sudo apt-get install -qq cmake binutils-dev python-pip

# Common backend libraries
sudo apt-get install -qq libboost-all-dev
sudo apt-get install -qq python-dev python-scipy python-numpy python-serial

# Visualization and graphical interfaces
sudo apt-get install -qq python-qt4-dev python-qt4-gl

# Tools
sudo apt-get install -qq git-lfs sshfs
sudo apt-get install -qq git-lfs gitk
git lfs install --skip-smudge

instlog "Installing common ROS dependencies"

# Hardware drivers
sudo apt-get install -qq ros-indigo-driver-base

# Cameras
sudo apt-get install -qq ros-indigo-usb-cam
sudo apt-get install -qq ros-indigo-camera-info-manager

# Lidar
sudo apt-get install -qq ros-indigo-hokuyo-node

# Navigation
sudo apt-get install -qq ros-indigo-robot-localization

# Image compression
sudo apt-get install -qq ros-indigo-rosbag-image-compressor
sudo apt-get install -qq ros-indigo-compressed-image-transport
sudo apt-get install -qq ros-indigo-compressed-depth-image-transport


#==========================#
# Finalization an Clean Up #
#==========================#

# Attempt to build the Navigator stack on client machines
if !(env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	instlog "Building the software stack with catkin_make"
	catkin_make -C "$CATKIN_DIR" -j8
fi

# Remove the initial install script if it was not in the Navigator repository
if !(echo "$SCRIPT_DIR" | grep --quiet "src/Navigator"); then
	rm -f "$SCRIPT_PATH"
fi
