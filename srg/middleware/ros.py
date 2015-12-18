"""

This file is part of FINITE STATE MACHINE BASED TESTING.

Copyright(c) <Florian Lier, Simon Schulz>
http://opensource.cit-ec.de/fsmt

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
import operator
import threading

# ROS IMPORTS
import rospy
import roslib
from std_msgs.msg import Header
from std_msgs.msg import String
from people_msgs.msg import Person
from people_msgs.msg import People
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point


# HLRC IMPORTS
from hlrc_client import RobotGaze


class RosControlConnector():

    def __init__(self):
        self.inscope = "/srg/arbitrate/toggle"
        self.stop_auto_arbitrate = False
        self.run = True
        t = threading.Thread(target=self.runner)
        t.start()

    def control_callback(self, ros_data):
        if ros_data.data.lower() == "pause":
            self.stop_auto_arbitrate = True
            print ">>> Auto Arbitrate is PAUSED"
        else:
            self.stop_auto_arbitrate = False
            print ">>> Auto Arbitrate is RESUMED"

    def runner(self):
        print ">>> Initializing ROS Toggle Subscriber to: %s" % self.inscope
        print "---"
        toggle_subscriber = rospy.Subscriber(self.inscope, String, self.control_callback, queue_size=1)
        while self.run is True:
            time.sleep(1.0)
        toggle_subscriber.unregister()
        print ">>> Deactivating Toggle Subscriber to: %s" % self.inscope


class RosConnector():
    """
    The GazeController receives person messages (ROS) and derives
    the nearest person identified. Based on this, the robot's
    joint angle target's are derived using the transformation
    class below
    """
    def __init__(self, _inscope, _transform, _datatype, _mode, _stimulus_timeout):
        self.run      = True
        self.trans    = _transform
        self.inscope  = str(_inscope).lower().strip()
        self.datatype = str(_datatype).lower().strip()
        self.mode     = str(_mode).lower().strip()
        self.stimulus_timeout = float(_stimulus_timeout)
        self.nearest_person_x = 0.0
        self.nearest_person_y = 0.0
        self.roi_x            = 0.0
        self.roi_y            = 0.0
        self.point_x          = 0.0
        self.point_y          = 0.0
        self.point_z          = 0.0
        self.current_robot_gaze = None
        self.current_robot_gaze_timestamp = None
        t = threading.Thread(target=self.runner)
        t.start()

    def people_callback(self, ros_data):
        send_time = ros_data.header.stamp
        idx = -1
        max_distance = {}
        for person in ros_data.people:
            idx += 1
            max_distance[str(idx)] = person.position.z
        # print ">> Persons found {idx, distance}: ", max_distance
        sort = sorted(max_distance.items(), key=operator.itemgetter(1), reverse=True)
        # print ">> Nearest Face: ", sort
        # print ">> Index: ", sort[0][0]
        # print ">> Distance in pixels: ", sort[0][1]
        self.nearest_person_x = ros_data.people[int(sort[0][0])].position.x
        self.nearest_person_y = ros_data.people[int(sort[0][0])].position.y
        # print ">> Position in pixels x:", self.nearest_person_x
        # print ">> Position in pixels y:", self.nearest_person_y
        point = [self.nearest_person_x, self.nearest_person_y]
        # Derive coordinate mapping
        angles = self.trans.derive_mapping_coords(point)
        if angles is not None:
            g = RobotGaze()
            if self.mode == 'relative':
                g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
            else:
                g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
            g.timestamp = send_time.to_sec()
            self.current_robot_gaze_timestamp = g.timestamp
            g.pan = angles[0]
            g.tilt = angles[1]
            self.current_robot_gaze = g
        self.honor_stimulus_timeout()

    def roi_callback(self, ros_data):
        # ROI MSGS don't feature a header, so we need to set
        # the timestamp here.
        send_time = time.time()
        self.roi_x = ros_data.regionofinterest.x_offset
        self.roi_y = ros_data.regionofinterest.y_offset
        point = [self.roi_x, self.roi_y]
        # Derive coordinate mapping
        angles = self.trans.derive_mapping_coords(point)
        if angles is not None:
            g = RobotGaze()
            if self.mode == 'absolute':
                g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
            else:
                g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
            g.timestamp = send_time.to_sec()
            self.current_robot_gaze_timestamp = g.timestamp
            g.pan = angles[0]
            g.tilt = angles[1]
            self.current_robot_gaze = g
        self.honor_stimulus_timeout()

    def point_callback(self, ros_data):
        send_time = ros_data.header.stamp
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
            g.timestamp = send_time.to_sec()
            self.current_robot_gaze_timestamp = g.timestamp
            g.pan = angles[0]
            g.tilt = angles[1]
            self.current_robot_gaze = g
        self.honor_stimulus_timeout()

    def honor_stimulus_timeout(self):
        time.sleep(self.stimulus_timeout)

    def runner(self):
        print ">>> Initializing ROS Subscriber to: %s" % self.inscope
        try:
            if self.datatype == "people":
                person_subscriber = rospy.Subscriber(self.inscope, People, self.people_callback, queue_size=1)
            elif self.datatype == "regionofinterest":
                person_subscriber = rospy.Subscriber(self.inscope, RegionOfInterest, self.roi_callback, queue_size=1)
            elif self.datatype == "pointstamped":
                person_subscriber = rospy.Subscriber(self.inscope, PointStamped, self.point_callback, queue_size=1)
            else:
                print ">>> ROS Subscriber DataType not supported %s" % self.datatype
                return
        except Exception, e:
            print ">>> ERROR %s" % str(e)
            return
        while self.run is True:
            time.sleep(1.0)
        person_subscriber.unregister()
        print ">>> Deactivating ROS Subscriber to: %s" % self.inscope