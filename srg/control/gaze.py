"""

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


class GazeController(threading.Thread):
    """
    The GazeController receives person messages (ROS) and derives
    the nearest person identified. Based on this, the robot's
    joint angle target's are derived using the transformation
    class below
    """
    def __init__(self, _robot_controller, _mw, _lock):
        threading.Thread.__init__(self)
        self.lock         = _lock
        self.mw           = _mw
        self.run_toggle   = True
        self.acquire_prio = False
        self.lastdatum    = time.time()
        self.rc           = _robot_controller
        self.loop_speed   = 1.0

    def run(self):
        print ">>> Initializing Gaze Controller for: %s --> %s" % (self.mw.inscope.strip(), self.rc.outscope.strip())
        loop_count = 0
        init_time = time.time()
        while self.run_toggle is True:
            then = time.time()
            tick = time.time()
            self.lock.acquire()
            if self.mw.current_robot_gaze is not None and self.lastdatum != self.mw.current_robot_gaze_timestamp:
                self.lastdatum = self.mw.current_robot_gaze_timestamp
                current_target = self.mw.current_robot_gaze
                if self.acquire_prio:
                    try:
                        print ">>> (setting_gaze)"
                        print current_target
                        self.rc.robot_controller.set_gaze_target(current_target, True)
                    except Exception, e:
                        print ">>> ERROR (set_gaze): %s" % str(e)
                    print "Set Gaze"
                    loop_count += 1
                self.lock.release()
            else:
                self.lock.release()
            now = time.time()
            # Running with maximum frequency of 50 Hz
            hz = 0.02-(now-then)
            if tick - init_time >= 1.0:
                self.loop_speed = loop_count
                loop_count = 0
                init_time = time.time()
            if hz > 0:
                time.sleep(hz)
        print ">>> Deactivating Gaze Controller for: %s" % self.rc.outscope.strip()
