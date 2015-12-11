# Simple Robot Gaze Based On HLRC

The Simple Robot Gaze component coordinates a Robot's gaze direction.
This can be done by a) providing (multiple) input sources and by b) prioritizing them.
For now, the component accepts ROS People messages [1] and ROS RegionOfInterest messsages [2]
then derives the corresponding robot joint angle configuration using the HLRC [3] library.
This is accomplished by mapping the region of interest (faces, saliency...) to the camera's
field of view and the robot's joint angles respectively.


# Gaze Strategy

This _simple_ approach first of all respects the provided priorities in the configuration file. If a stimulus, a ROI
for instance, is sent via middleware input stream and if it is not 'older' than 2 seconds (timestamp of message header)
than, the highest priority input stream will be granted access to the gaze controller and will subsequently to control
the robot. If the current stimulus is older than 2 seconds, the next higher priority input stream will be checked for
new input. If there is no input at all, or all input is decayed (older than 2 seconds) the highest priority input stream
(last provided value) will always granted access.


## Recommended Installation Method:

Please use the CITK to install this component, there is already a project file for this: hlrc-simple-robot-gaze.project


## Manual Install

    python setup.py


## Usage

    simple_robot_gaze -c config_file -o /hlrc_control_scope

    Example: simple_robot_gaze -c ${HOME}/.config/simplerobotgaze.yaml -o /flobi


## Config File Explained

The file must reside in ~/.config/simplerobotgaze.yaml

In order to stop simple robot gaze from controlling the robot simple send "pause" (string msg) to:

    /srg/arbitrate/toggle

If you send something other than "pause", simple robot gaze will resume operation.

The priority of input data streams, the first entry has the highest priority, the last the lowest.

priorities:
  - /ocvfacerec/ros/people
  - /dlibfacerec/ros/people
  - /movement/roi

Gaze strategy: relative for moving cameras, absolute for fixed setups, corresponds to the priorities.

modes:
  - relative
  - relative
  - absolute

What kind of data are you sending in your input stream, corresponds to priorities.

datatypes:
  - ros:People
  - ros:RegionOfInterest
  - ros:RegionOfInterest

Resolution of the source camera image, corresponds to the priorities.

resolution:
  - 320x240
  - 320x240
  - 320x240

Camera field of view horizontal and vertical (in degree), also corresponds to the priorities.

fov:
  - 58.0x45.0
  - 58.0x45.0
  - 58.0x45.0


# TODO

Implement RSB support for input scopes and remote control.