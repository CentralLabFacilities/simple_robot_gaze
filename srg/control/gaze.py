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
import threading


class GazeController():
    """
    The GazeController receives person messages (ROS) and derives
    the nearest person identified. Based on this, the robot's
    joint angle target's are derived using the transformation
    class below
    """
    def __init__(self, _robot_controller, _mw):
        print(">>> Initializing Gaze Controller")
        self.mw       = _mw
        self.run      = True
        self.rc       = _robot_controller
        signal.signal(signal.SIGINT, self.signal_handler)
        t = threading.Thread(target=self.runner)
        t.start()
        t.join()

    def signal_handler(self, signal, frame):
            print ">>> ROS Connector is about to exit (signal %s)..." % str(signal)
            self.mw.run = False
            self.run = False

    def runner(self):
        while self.run is True:
            if self.mw.current_robot_gaze is not None:
                current_target = self.mw.current_robot_gaze
                self.rc.robot_controller.set_gaze_target(current_target, True)
            # Running at 100 Hz Maximum!
            time.sleep(0.01)