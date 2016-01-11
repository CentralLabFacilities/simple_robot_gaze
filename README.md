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

In order to pause simple robot gaze from controlling the robot simple send "true" to (resume = "false"):

    /robotgaze/set/pause

In order to get the current status subscribe (return true or false) to:

    /robotgaze/get/pause

The priority of input data streams, the first entry has the highest priority, the last the lowest.

priorities:
  - /robotgazetools/faces
  - /robotgazetools/saliency

What kind of data are you sending in your input stream, corresponds to priorities. Currently implemented: ros:People, ros:PointStamped

datatypes:
  - ros:People
  - ros:PointStamped

Gaze strategy: relative for moving cameras, absolute for fixed setups, corresponds to the priorities

modes:
  - relative
  - absolute

Skip stimulus input x for n seconds. For example: only react every three seconds on stimulus input would be "3.0"
This corresponds to priorities. The highest allowed value _must_ be less than "boring_timeout" (see below).

stimulus_timeout:
  - 0.0
  - 0.0

Based on the input streams (see: priorities). When is a stimulus considered "boring", e.g., no new messages for n
seconds. The last received timestamp is always evaluated.
For example: time.now() - timestamp_last_message >= boring_timeout: proceed to the next priority level.

boring_timeout:
  - 1.0

See "peak_override"

allow_peak_override:
  - 1

Peak override: Provide a value that is encoded in your stimulus input messages, e.g, size of face in pixels in order
to override the base priotrity. This field is only evaluated if allow_peak_override is "1" (see above).
Example: your first priority is facedetection. However, if there is massive motion detected in the second priority,
override the first priority. Corresponds priorities.

peak_overrides:
  - 100.0
  - 50.0

Resolution of the source camera image, corresponds to the priorities

resolution:
  - 320x240
  - 320x240

Camera field of view horizontal and vertical (in degree), also corresponds to the priorities.
fov:
  - 66.0x40.0
  - 66.0x40.0