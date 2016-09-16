# IEEE2017

Email Matthew Langford at matthew.langford95@gmail.com if you are interested in helping out or if you want information about the project.

Join the [Facebook page](https://www.facebook.com/groups/489613054499546/) for important news and meeting time updates.

The [competition rules](http://sites.ieee.org/southeastcon2017/files/2016/04/MMXVII.pdf) contain all of the information that will be needed to design and build our robot.

# Getting involved

In order to develop software for the robot, you will need to become familiar with a few tools that are the building blocks of our codebase and workflow:
* [Ubuntu](http://www.ubuntu.com/) - This is the linux-based operating system that we use. Currently, only Ubuntu 14.04 is supported by our software stack, so that is the version you will need. If you are not familiar with Linux and Bash, I recommend you read through this [tutorial](http://ryanstutorials.net/linuxtutorial/).
* [Python](https://www.python.org/) - This is the main programming language used on the project and the one recommended to new programmers. C++ is also an option if you feel more comfortable with it, but the learning curve to use it with ROS is a bit steeper.
* [ROS](http://www.ros.org/) - The Robot Operating System is what we use to ensure that nodes running on the robot can communicate properly. For more information, we recommend visiting the site and reading through a couple of the tutorials.
* [Git](https://git-scm.com/) - Git is our version control software; it allows us to keep track of the history of files so that we know who made which changes and revert back to previous versions if neccessary. We use a [forking workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow) to facilitate contributions. For a beginner's introduction to git, see this [tutorial](https://git-scm.com/doc).
* [OpenCV](http://opencv.org/) - This library enables us to perform complex robot vision tasks in a simple and elegant way. If you are intrested in robot vision, this will be your go to.

# Setting Up the Development Environment

IEEE2017 has many dependencies that may be cumbersome to install by hand. A custom image of Ubuntu has been prepared to run in a virtual environment using the install script. The recommended tool to run the image is [VirtualBox](https://www.virtualbox.org/), because that is what it was created in. VMWare is also an option if you have a strong preference to it. Either way, you should simply be able to import the virtual appliance. The OVA file is available [here](http://subjugator.org/extfiles/IEEE2017-VM.ova). The md5sum hash of the file is *ecfb1c60bbe1c6c513d4607b54e00ba6* (please ensure that the md5sum of the file you downloaded matches for security purposes).

There is also a convenient install script to fetch and install them. This script can be run on Ubuntu if you would prefer to set up your own operating system or already have Ubuntu installed. If the default install location of the catkin workspace (~/ieee_ws) is not ideal, then a different path can be passed to the script with the -c option.

#### If you have never cloned this repository or done work with the hardware team

The following commands should be run:

    curl -s https://raw.githubusercontent.com/whispercoros/IEEE2017/master/install.sh | sudo bash

#### If you have already cloned this repository or done work with the hardware team

The following commands should be run:

    ~/ieee_ws/src/IEEE2017/install.sh

Make sure that this is actually the path to the local catkin workspace and install script before running it!

The install script is intended to handle *every single thing* that needs to be installed to develop for NaviGator. If it does not work, something has gone wrong and it needs to be fixed. If an issue is fixed while installing, please fix the install script and submit a pull-request with the changes.

The install script can accept arguments. For example, `./install.sh -c ~/ieee2017_ws` will generate a catkin workspace with the NaviGator repository in it at `~/ieee2017_ws` or use an existing workspace if one exists at that location. It will make all of the files and directories it needs within that workspace. If the script has previously been run, it will not run initial set up tasks that have already been performed unless they need to be changed.
