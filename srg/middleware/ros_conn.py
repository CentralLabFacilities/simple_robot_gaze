"""

Copyright(c) <Florian Lier>


This file may be licensed under the terms of the
GNU Lesser General Public License Version 3 (the ``LGPL''),
or (at your option) any later version.

Software distributed under the License is distributed
on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
express or implied. See the LGPL for the specific language
governing rights and limitations.

You should have received a copy of the LGPL along with this
program. If not, go to http://www.gnu.org/licenses/lgpl.html
or write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

The development of this software was supported by the
Excellence Cluster EXC 277 Cognitive Interaction Technology.
The Excellence Cluster EXC 277 is a grant of the Deutsche
Forschungsgemeinschaft (DFG) in the context of the German
Excellence Initiative.

Authors: Florian Lier, Simon Schulz
<flier, sschulz>@techfak.uni-bielefeld.de

"""

# STD IMPORTS
import time
import math
import operator
import threading

# ROS IMPORTS
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import String
from people_msgs.msg import Person
from geometry_msgs.msg import Pose
from people_msgs.msg import People
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerPose

# HLRC IMPORTS
from hlrc_client import RobotGaze
from hlrc_client import RobotTimestamp

# NUMPY
import numpy as np


class ROSPausePublisher(threading.Thread):

    def __init__(self, _prefix, _paused, _lock):
        threading.Thread.__init__(self)

        self.prefix       = "/"+str(_prefix.lower().strip())
        self.paused       = _paused
        self.is_paused    = False
        self.lock         = _lock
        self.run_toggle   = True
        self.rate         = rospy.Rate(20)
        self.pub_setpause = rospy.Publisher(self.prefix+"/robotgaze/set/pause", Bool, queue_size=10)
        self.pub_getpause = rospy.Publisher(self.prefix+"/robotgaze/get/pause", Bool, queue_size=10)

        self.p = True
        self.r = False

    def pause(self):
        self.pub_setpause.publish(self.p)

    def resume(self):
        self.pub_setpause.publish(self.r)

    def run(self):
        print ">>> Initializing ROS Pause Publisher to: %s" % self.prefix+"/robotgaze/get/pause"
        while self.run_toggle is True:
            self.lock.acquire(1)
            self.pub_getpause.publish(self.paused.get_paused())
            self.is_paused = self.paused.get_paused()
            self.lock.release()
            self.rate.sleep()
        print ">>> Deactivating ROS Pause Publisher to: %s" % self.prefix+"/robotgaze/get/pause"


class ROSPauseControlConnector(threading.Thread):

    def __init__(self, _prefix, _paused, _lock):
        threading.Thread.__init__(self)

        self.run_toggle = True
        self.lock       = _lock
        self.paused     = _paused
        self.prefix     = "/"+str(_prefix.lower().strip())
        self.inscope    = self.prefix+"/robotgaze/set/pause"

    def control_callback(self, ros_data):
        if ros_data.data is True:
            self.lock.acquire(1)
            self.paused.set_pause()
            self.lock.release()
            print ">>> Auto Arbitrate is PAUSED (ROS)"
        else:
            self.lock.acquire(1)
            self.paused.set_resume()
            self.lock.release()
            print ">>> Auto Arbitrate is RESUMED (ROS)"

    def run(self):
        print ">>> Initializing ROS Pause Subscriber to: %s" % self.inscope.strip()
        pause_subscriber = rospy.Subscriber(self.inscope, Bool, self.control_callback, queue_size=10)
        while self.run_toggle is True:
            time.sleep(0.05)
        pause_subscriber.unregister()
        print ">>> Deactivating ROS Pause Subscriber to: %s" % self.inscope.strip()


class ROSStatusConnector:

    def __init__(self, _prefix):
        self.prefix     = "/"+str(_prefix.lower().strip())
        self.run_toggle = True
        self.pub_status = rospy.Publisher(self.prefix+"/robotgaze/status", String, queue_size=10)
        print ">>> Initializing ROS Status Publisher to: %s" % self.prefix+"/robotgaze/status"

    def publish_status_info(self, _status):
        self.pub_status.publish(str(_status))


class ROSSetDirectGazeConnector(threading.Thread):

    def __init__(self, _prefix, _rd):
        threading.Thread.__init__(self)

        self.run_toggle      = True
        self.ready           = False
        self.robot_driver    = _rd
        self.point_x         = 0.0
        self.point_y         = 0.0
        self.point_z         = 0.0
        self.last_robot_gaze = None
        self.last_time_stamp = time.time()
        self.prefix          = "/"+str(_prefix.lower().strip())
        self.inscope         = self.prefix+"/robotgaze/set/gaze"

    def direct_gaze_callback(self, ros_data):
        send_time    = ros_data.header.stamp
        self.point_x = ros_data.point.x
        self.point_y = ros_data.point.y
        self.point_z = ros_data.point.z
        point = [self.point_x, self.point_y]
        if point is not None and self.last_time_stamp != send_time:
            g = RobotGaze()
            g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
            g.gaze_timestamp = RobotTimestamp(send_time.to_sec())
            g.pan  = point[0]
            g.tilt = point[1]
            g.roll = 0.0
            self.last_robot_gaze = g
            self.robot_driver.robot_controller.set_gaze_target(g, True)
            self.last_time_stamp = send_time
            print ">>> Direct Gaze (ROS) set to: %f, %f" % (g.pan, g.tilt)

    def run(self):
        print ">>> Initializing ROS Direct Gaze Subscriber to: %s" % self.inscope.strip()
        toggle_subscriber = rospy.Subscriber(self.inscope, PointStamped, self.direct_gaze_callback, queue_size=10)
        self.ready = True
        while self.run_toggle is True:
            time.sleep(0.05)
        toggle_subscriber.unregister()
        print ">>> Deactivating ROS Direct Gaze Subscriber to: %s" % self.inscope.strip()


class ROSToggleConnector(threading.Thread):

    def __init__(self, _prefix, _inscope ,_subscriber):
        threading.Thread.__init__(self)

        self.run_toggle = True
        self.toggle     = False
        self.inscope    = _inscope
        self.subscriber = _subscriber
        self.scope      = "/"+str(_prefix.lower().strip())+"/robotgaze/"+str(_inscope.lower().strip())+"/toggle"

    def toggle_callback(self, ros_data):
        if ros_data.data is True:
            self.subscriber.unregister()
            print ">>> Subscriber (ROS) for %s unsubscribed" % self.inscope
        else:
            print ">>> Not implemented" % self.inscope
            # print ">>> Subscriber (ROS) for %s re-subscribed" % self.inscope

    def run(self):
        print ">>> Initializing ROS Toggle Subscriber to: %s" % self.scope
        toggle_subscriber = rospy.Subscriber(self.inscope, Bool, self.toggle_callback, queue_size=10)
        while self.run_toggle is True:
            time.sleep(0.05)
        toggle_subscriber.unregister()
        print ">>> Deactivating ROS Pause Subscriber to: %s" % self.scope


class ROSDataConnector(threading.Thread):
    """
    The GazeController receives person messages (ROS) and derives
    the nearest person identified. Based on this, the robot's
    joint angle target's are derived using the transformation
    class below
    """
    def __init__(self, _prefix, _inscope, _transform, _datatype, _mode, _stimulus_timeout, _lock):
        threading.Thread.__init__(self)
        self.run_toggle = True
        self.ready      = False
        self.lock       = _lock
        self.trans      = _transform
        self.subscriber = None
        self.prefix     = str(_prefix).lower().strip()
        self.mode       = str(_mode).lower().strip()
        self.inscope    = str(_inscope).lower().strip()
        self.datatype   = str(_datatype).lower().strip()

        self.current_robot_gaze = None
        self.stimulus_timeout   = float(_stimulus_timeout)
        self.current_robot_gaze_timestamp = None

        self.nearest_person_x = 0.0
        self.nearest_person_y = 0.0
        self.nearest_person_z = 0.0
        self.roi_x            = 0.0
        self.roi_y            = 0.0
        self.point_x          = 0.0
        self.point_y          = 0.0
        self.point_z          = 0.0
        self.pan              = 0.0
        self.tilt             = 0.0
        self.roll             = 0.0

    def people_callback(self, ros_data):
        self.lock.acquire(1)
        send_time = ros_data.header.stamp
        idx = -1
        max_distance = {}
        for person in ros_data.people:
            idx += 1
            max_distance[str(idx)] = person.position.z
        sort = sorted(max_distance.items(), key=operator.itemgetter(1), reverse=True)
        self.nearest_person_x = ros_data.people[int(sort[0][0])].position.x
        self.nearest_person_y = ros_data.people[int(sort[0][0])].position.y
        self.nearest_person_z = ros_data.people[int(sort[0][0])].position.z
        point = [self.nearest_person_x, self.nearest_person_y]
        # Derive coordinate mapping
        angles = self.trans.derive_mapping_coords(point)
        if angles is not None:
            g = RobotGaze()
            if self.mode == 'relative':
                g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
            else:
                g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
            self.current_robot_gaze_timestamp = send_time.to_sec()
            g.gaze_timestamp = RobotTimestamp(self.current_robot_gaze_timestamp)
            g.pan  = angles[0]
            g.tilt = angles[1]
            g.roll = self.roll
            self.current_robot_gaze = g
        self.lock.release()
        self.honor_stimulus_timeout()

    def point_callback(self, ros_data):
        self.lock.acquire(1)
        send_time    = ros_data.header.stamp
        self.point_x = ros_data.point.x
        self.point_y = ros_data.point.y
        self.point_z = ros_data.point.z
        point = [self.point_x, self.point_y]
        # Derive coordinate mapping
        angles = self.trans.derive_mapping_coords(point)
        if angles is not None:
            g = RobotGaze()
            if self.mode == 'absolute':
                g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
            else:
                g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
            self.current_robot_gaze_timestamp = send_time.to_sec()
            g.gaze_timestamp = RobotTimestamp(self.current_robot_gaze_timestamp)
            g.pan  = angles[0]
            g.tilt = angles[1]
            g.roll = self.roll
            self.current_robot_gaze = g
        self.lock.release()
        self.honor_stimulus_timeout()

    def marker_callback(self, ros_data):
        self.lock.acquire(1)
        if len(ros_data.markers) > 0:
            send_time = ros_data.markers[0].header.stamp
            p = ros_data.markers[0].pose
            self.point_x = p.position.x
            self.point_y = p.position.y
            self.point_z = p.position.z
            vector_z = np.array([0.0, 0.0, self.point_z])
            vector_x = np.array([self.point_x, 0.0, self.point_z])
            vector_y = np.array([0.0, self.point_y, self.point_z])
            # NOTE: Bart Convention
            self.pan = self.trans.angle_between(vector_z, vector_x, "pan")
            self.tilt = self.trans.angle_between(vector_z, vector_y, "tilt")
            if not math.isnan(self.pan) and not math.isnan(self.tilt):
                g = RobotGaze()
                if self.mode == 'absolute':
                    g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
                else:
                    g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
                self.current_robot_gaze_timestamp = send_time.to_sec()
                g.gaze_timestamp = RobotTimestamp(self.current_robot_gaze_timestamp)
                g.pan = self.pan
                g.tilt = self.tilt
                g.roll = self.roll
                self.current_robot_gaze = g
        self.lock.release()
        self.honor_stimulus_timeout()

    def interactive_marker_callback(self, ros_data):
        self.lock.acquire(1)
        send_time = ros_data.header.stamp
        p = ros_data.pose
        self.point_x = p.position.x
        self.point_y = p.position.y
        self.point_z = p.position.z
        point = [self.point_x, self.point_y]
        # Derive coordinate mapping
        angles = self.trans.derive_mapping_coords(point)
        if angles is not None:
            g = RobotGaze()
            if self.mode == 'relative':
                g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
            else:
                g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
            self.current_robot_gaze_timestamp = send_time.to_sec()
            g.gaze_timestamp = RobotTimestamp(self.current_robot_gaze_timestamp)
            g.pan = angles[0]
            g.tilt = angles[1]
            g.roll = self.roll
            self.current_robot_gaze = g
        self.lock.release()
        self.honor_stimulus_timeout()

    def honor_stimulus_timeout(self):
        time.sleep(self.stimulus_timeout)

    def run(self):
        print ">>> Initializing ROS Data Subscriber to: %s" % self.inscope.strip()
        try:
            if self.datatype == "people":
                self.subscriber = rospy.Subscriber(self.inscope, People, self.people_callback, queue_size=10)
            elif self.datatype == "pointstamped":
                self.subscriber = rospy.Subscriber(self.inscope, PointStamped, self.point_callback, queue_size=10)
            elif self.datatype == "markerarray":
                self.subscriber = rospy.Subscriber(self.inscope, MarkerArray, self.marker_callback, queue_size=10)
            elif self.datatype == "interactivemarkerpose":
                self.subscriber = rospy.Subscriber(self.inscope, InteractiveMarkerPose, self.interactive_marker_callback, queue_size=10)
            else:
                print ">>> ROS Data Subscriber DataType not supported %s" % self.datatype.strip()
                return
        except Exception, e:
            print ">>> ERROR %s" % str(e)
            return
        self.ready = True
        while self.run_toggle is True:
            time.sleep(0.05)
        self.subscriber.unregister()
        print ">>> Deactivating ROS Data Subscriber to: %s" % self.inscope.strip()
