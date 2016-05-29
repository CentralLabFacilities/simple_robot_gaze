# Simple Robot Gaze Config File

The Simple Robot Gaze component coordinates a Robot's gaze direction.
This can be done by a) providing (multiple) input sources and by b) prioritizing them.
For now, the component accepts ROS People messages [1] and ROS RegionOfInterest messsages [2]
then derives the corresponding robot joint angle configuration using the HLRC [3] library.
This is accomplished by mapping from the region of interest (faces, ittikoch...) to the camera's
field of view, robot joint angles respectively.

This client features various remote control functions, these are usually available
under /$scope_topic_prefix/robotgaze/$something . The default is /robot/robotgaze/$something

    scope_topic_prefix:
        - robot

In order to pause robot gaze from controlling the robot simply send "true" (bool msg) to:

    $scope_topic_prefix/robotgaze/set/pause using either RSB or ROS

In order to directly send gaze targets send PointStamped msgs (ROS) or SphericalDirectionFloat (RSB) to:

    $scope_topic_prefix/robotgaze/set/gaze

ROS remote control support is enabled by default. If you wish to control SRG remotely via
RSB set this to "1". If you don't know what RSB is, leave this value "0"

    enable_rsb_remote_control:
        - 0

The priority of input data streams, the first entry has the highest priority

    priorities:
        - /robotgazetools/faces
        - /robotgazetools/saliency

The control loop field allows to set the control strategy to be set either "open", which means
no robot feedback is taken into account when sending gaze targets (fire and forget). Or, if set, i.e.,
srgplugins.mekarobot SRG will automatically load the provided plugin and wait for the next target to be
sent if, and only if, the desired position (provided by the loaded plugin) has been reached (default timeout is 10 seconds.).
Plugin Documentation: https://github.com/CentralLabFacilities/simple_robot_gaze_plugins.
If you are unsure leave this value "open"

    control_loop:
        - open
        - srgplugins.mekarobot

What kind of data are you sending via your input stream, corresponds to priorities. Currently implemented:
ROS: ros:People, ros:PointStamped, ros:MarkerArray, ros:InteractiveMarker
RSB: rsb:Faces, rsb:SphericalDirectionFloat

    datatypes:
    - ros:People
    - rsb:SphericalDirectionFloat

Resolution of the data source, corresponds to the priorities. NOTE: In case of an MarkerArray msg, just copy an
existing value or simple write 1x1 since the marker position needs NO transformation.

    resolution:
        - 320x240
        - 320x240

Robot's field of view horizontal and vertical (in degree), also corresponds to the priorities. NOTE: In case of an 
MarkerArray msg, just copy an existing value or simple write 1x1 since the marker position needs NO transformation.

    fov:
        - 66.0x40.0
        - 66.0x40.0

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
override the first priority. Corresponds priorities. ":> or :<" is to be interpreted as larger than or lower than number.

    peak_overrides:
        - 100.0:>
        - 10.0:<