# IEEE2017

Email Arghya Das at argho33@ufl.edu if you are interested in helping out or if you want information about the project.

The [competition rules](http://sites.ieee.org/southeastcon2017/student-program) contain all of the information that will be needed to design and build our robot.

# Getting involved

In order to develop software for the robot, you will need to become familiar with a few tools that are the building blocks of our codebase and workflow:
* [Ubuntu](http://www.ubuntu.com/) - This is the linux-based operating system that we use. Currently, only Ubuntu 14.04 is supported by our software stack, so that is the version you will need. If you are not familiar with Linux and Bash, I recommend you read through this [tutorial](http://ryanstutorials.net/linuxtutorial/).
* [Python](https://www.python.org/) - This is the main programming language used on the project and the one recommended to new programmers. C++ is also an option if you feel more comfortable with it, but the learning curve to use it with ROS is a bit steeper.
* [ROS](http://www.ros.org/) - The Robot Operating System is what we use to ensure that nodes running on the robot can communicate properly. For more information, we recommend visiting the site and reading through a couple of the tutorials.
* [Git](https://git-scm.com/) - Git is our version control software; it allows us to keep track of the history of files so that we know who made which changes and revert back to previous versions if neccessary. We use a [forking workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow) to facilitate contributions. For a beginner's introduction to git, see this [tutorial](https://git-scm.com/doc).
* [OpenCV](http://opencv.org/) - This library enables us to perform complex robot vision tasks in a simple and elegant way. If you are intrested in robot vision, this will be your go to.

# Setting Up the Development Environment

IEEE2017 has many dependencies that may be cumbersome to install by hand. A custom image of Ubuntu has been prepared to run in a virtual environment using the install script. The recommended tool to run the image is [VirtualBox](https://www.virtualbox.org/), because that is what it was created in. VMWare is also an option if you have a strong preference to it. Either way, you should simply be able to import the virtual appliance. The OVA file is available [here](http://subjugator.org/extfiles/IEEE2017-VM.ova). The md5sum hash of the file is **ecfb1c60bbe1c6c513d4607b54e00ba6** (please ensure that the md5sum of the file you downloaded matches for security purposes).

There is also a convenient install script to fetch and install them. This script can be run on Ubuntu if you would prefer to set up your own operating system or already have Ubuntu installed.

The following command will fetch and run the script:

    sudo apt-get -qq update && sudo apt-get install -qq curl && bash <(curl -s https://raw.githubusercontent.com/ufieeehw/IEEE2017/master/install.sh) 

The install script is intended to handle *every single thing* that needs to be installed to develop for the robot. If it does not work, something has gone terribly wrong and it needs to be fixed. If you resolve an issue while installing, please fix it in the install script and submit a pull-request with the changes. Otherwise, notify the script's maintainer (currently [Anthony Olive](https://github.com/whispercoros)).

The script will create all of the files and directories it needs within the selected catkin workspace. If the script has previously been run, it will not run initial set up tasks that have already been performed unless they need to be updated. This means that the script will respect workspaces with git repositories already present in them.
