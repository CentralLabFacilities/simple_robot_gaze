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
import signal
import operator

# HLRC IMPORTS
from hlrc_client import RobotGaze as rg

# ROS IMPORTS
import rospy
import roslib
from std_msgs.msg import Header
from std_msgs.msg import String
from people_msgs.msg import Person
from people_msgs.msg import People


class GazeController():
    """
    The GazeController receives person messages (ROS) and derives
    the nearest person identified. Based on this, the robot's
    joint angle target's are derived using the transformation
    class below
    """
    def __init__(self, _robot_controller, _affine_transform, _inscope):
        print(">>> Initializing Gaze Controller")
        self.run      = True
        self.inscope  = _inscope
        self.rc       = _robot_controller
        self.at       = _affine_transform
        self.nearest_person_x = 0.0
        self.nearest_person_y = 0.0
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signal, frame):
            print ">>> ROS is about to exit (signal %s)..." % str(signal)
            self.run = False

    def people_callback(self, ros_data):
        # Determine the nearest person
        idx = -1
        max_distance = {}
        for person in ros_data.people:
            idx += 1
            max_distance[str(idx)] = person.position.z
        print ">> Persons found {idx, distance}: ", max_distance
        sort = sorted(max_distance.items(), key=operator.itemgetter(1), reverse=True)
        print ">> Nearest Face: ", sort
        print ">> Index: ", sort[0][0]
        print ">> Distance in pixels: ", sort[0][1]
        self.nearest_person_x = ros_data.people[int(sort[0][0])].position.x
        self.nearest_person_y = ros_data.people[int(sort[0][0])].position.y
        send_time = ros_data.header.stamp
        print ">> Position in pixels x:", self.nearest_person_x
        print ">> Position in pixels y:", self.nearest_person_y
        point = [self.nearest_person_x, self.nearest_person_y]
        # Derive coordinate mapping
        angles = self.at.derive_mapping_coords(point)
        print "----------------"
        if angles is not None:
            # Set the robot gaze
            g = rg.RobotGaze()
            g.gaze_type = rg.RobotGaze.GAZETARGET_RELATIVE
            g.timestamp = send_time.to_sec()
            g.pan = angles[0]
            g.tilt = angles[1]
            print ">> Sending Gaze Type:", g
            self.rc.robot_controller.set_gaze_target(g, False)

    def run_subscriber(self):
        print(">>> Initializing Gaze Subscriber")
        person_subscriber = rospy.Subscriber(self.inscope, People, self.people_callback, queue_size=1)
        while self.run:
            time.sleep(1)
        person_subscriber.unregister()
        print ">>> Deactivating ROS Subscriber"

    def derive_gaze_angle(self):
        pass