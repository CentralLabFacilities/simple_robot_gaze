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
import threading


class GazeController():
    """
    The GazeController receives person messages (ROS) and derives
    the nearest person identified. Based on this, the robot's
    joint angle target's are derived using the transformation
    class below
    """
    def __init__(self, _robot_controller, _mw):
        self.mw           = _mw
        self.run          = True
        self.lastdatum    = time.time()
        self.rc           = _robot_controller
        self.acquire_prio = False
        t = threading.Thread(target=self.runner)
        t.start()

    def runner(self):
        print ">>> Initializing Gaze Controller to: %s" % self.rc.outscope
        print "---"
        while self.run is True:
            if self.mw.current_robot_gaze is not None and self.lastdatum != self.mw.current_robot_gaze.timestamp:
                current_target = self.mw.current_robot_gaze
                self.lastdatum = self.mw.current_robot_gaze.timestamp
                if self.acquire_prio:
                    self.rc.robot_controller.set_gaze_target(current_target, True)
            else:
                time.sleep(0.005)
        print">>> Deactivating Gaze Controller to: %s" % self.rc.outscope