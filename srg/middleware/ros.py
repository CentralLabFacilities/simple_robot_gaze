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

# HLRC IMPORTS
from hlrc_client import RobotGaze


class RosConnector():
    """
    The GazeController receives person messages (ROS) and derives
    the nearest person identified. Based on this, the robot's
    joint angle target's are derived using the transformation
    class below
    """
    def __init__(self, _inscope, _transform, _datatype):
        self.run      = True
        self.trans    = _transform
        self.inscope  = str(_inscope).strip()
        self.datatype = str(_datatype).lower().strip()
        self.nearest_person_x = 0.0
        self.nearest_person_y = 0.0
        self.roi_x            = 0.0
        self.roi_y            = 0.0
        self.current_robot_gaze = None
        t = threading.Thread(target=self.runner)
        t.start()

    def people_callback(self, ros_data):
        # Determine the nearest person
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
        send_time = ros_data.header.stamp
        # print ">> Position in pixels x:", self.nearest_person_x
        # print ">> Position in pixels y:", self.nearest_person_y
        point = [self.nearest_person_x, self.nearest_person_y]
        # Derive coordinate mapping
        angles = self.trans.derive_mapping_coords(point)
        # print "----------------"
        if angles is not None:
            g = RobotGaze()
            g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
            g.timestamp = send_time.to_sec()
            g.pan = angles[0]
            g.tilt = angles[1]
            self.current_robot_gaze = g

    def roi_callback(self, ros_data):
        send_time = ros_data.header.stamp
        self.roi_x = ros_data.regionofinterest.x_offset
        self.roi_y = ros_data.regionofinterest.y_offset
        point = [self.roi_x, self.roi_y]
        # Derive coordinate mapping
        angles = self.trans.derive_mapping_coords(point)
        # print "----------------"
        if angles is not None:
            g = RobotGaze()
            g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
            g.timestamp = send_time.to_sec()
            g.pan = angles[0]
            g.tilt = angles[1]
            self.current_robot_gaze = g

    def runner(self):
        print ">>> Initializing ROS Subscriber to: %s" % self.inscope
        print "---"
        if self.datatype == "people":
            person_subscriber = rospy.Subscriber(self.inscope, People, self.people_callback, queue_size=1)
        elif self.datatype == "regionofinterest":
            person_subscriber = rospy.Subscriber(self.inscope, RegionOfInterest, self.roi_callback, queue_size=1)
        else:
            print ">>> ROS Subscriber DataType not supported %s" % self.datatype
            return
        while self.run is True:
            time.sleep(1)
        person_subscriber.unregister()
        print ">>> Deactivating ROS Subscriber to: %s" % self.inscope