#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
sudo cp $DIR/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
