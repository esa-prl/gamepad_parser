# Gamepad Parser

## Overview

This package maps the inputs from a gamepad into commands for the rover motion, the locomotion modes and the PTU orientation.

**Keywords:** joystick, gamepad, mapping, package

### License

The source code is released under a [TODO: Add Licence]()).

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.com**

The Gamepad Parser package has been tested under [ROS2] Eloquent and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rover_msgs] (message definitions for ESA-PRL rovers)

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/gamepad_parser.git
	cd ../
	colcon build --symlink-install

Adding `--symlink-install` to `colcon build` eliminates the need to recompile the package after changing the code.


## Usage

Run the main node with:

	ros2 run gamepad_parser gamepad_parser_node

and add a joystick with:

	ros2 run joy joy_node --ros-args --remap /joy:=/gamepad

## Config files

Config file config/

* **gamepad_parser.yaml** 
	- **`deadzone`** Zone where signal of the analog stick is ignored. - `[0,1]`

## Nodes

### gamepad_parser_node

Maps gamepad inputs to rover velocities, PTU commands and locomotion mode change requests.

#### Subscribed Topics

* **`/gamepad`** ([sensor_msgs/Joy])

	Input of the buttons and joystick of a gamepad.	

#### Published Topics

* **`/rover_motion_cmd`** ([geometry_msgs/Twist])

	Command containing Linear and angular velocities for the rover.


* **`/ptu_cmd`** ([geometry_msgs/Twist])

	Command containing Linear and angular velocities for the PTU.


#### Services

* **`change_locomotion_mode`** ([rover_msgs/ChangeLocomotionMode])

	Requests a new locomotion mode from the locomotion manager.

#### Parameters

TODO

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rover_msgs/ChangeLocomotionMode]: https://github.com/esa-prl/rover_msgs/blob/master/srv/ChangeLocomotionMode.srv
[sensor_msgs/Joy]: http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html
[geometry_msgs/Twist]: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html