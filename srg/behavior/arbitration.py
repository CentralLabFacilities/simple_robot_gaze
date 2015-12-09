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
import sys
import time
import yaml
import threading

# SELF IMPORTS
from srg.robot import driver as d
from srg.control import gaze as g
from srg.middleware import ros as r
from srg.utils import transform as t


class Arbitration:

    def __init__(self, _configfile, _outscope):
        self.run             = True
        self.cfgfile         = _configfile.strip()
        self.config          = None
        self.transforms      = []
        self.input_sources   = []
        self.gaze_controller = []
        self.boring          = 2.0
        # Robot Control
        self.rd = d.RobotDriver("ROS", _outscope.strip())
        time.sleep(0.5)
        self.read_yaml_config()
        self.configure_middleware()
        # Start Arbitration
        t = threading.Thread(target=self.arbitrate)
        t.start()

    def request_stop(self):
        self.run = False
        time.sleep(0.2)
        for connection in self.input_sources:
            connection.run = False
        for gazecontrol in self.gaze_controller:
            gazecontrol.run = False

    def read_yaml_config(self):
        try:
            print ">>> Using config: %s" % self.cfgfile
            print "---"
            f = open(self.cfgfile)
            config_vals = yaml.load(f)
            self.config = config_vals
            f.close()
            if len(self.config["resolution"]) != len(self.config["priorities"]) or len(self.config["resolution"]) != len(self.config["fov"]):
                print ">>> Please check your config file, not enough values provided..."
                sys.exit(1)
            else:
                pass
        except Exception, e:
            print ">>> %s" % str(e)
            sys.exit(1)

    def configure_middleware(self):
        idx = 0
        for item in self.config["priorities"]:
            # Read config file an extract values
            res        = self.config["resolution"][idx].split("x")
            fov        = self.config["fov"][idx].split("x")
            datatypes  = self.config["datatypes"][idx].split(":")
            # Transformations
            at = t.AffineTransform(str(item))
            at.set_coords(float(res[0]), float(res[1]), float(fov[0]), float(fov[1]))
            at.calculate_divider()
            self.transforms.append(at)
            time.sleep(0.1)
            # Middleware
            if datatypes[0].lower() == "ros":
                mw = r.RosConnector(str(item), at, datatypes[1])
            elif datatypes[0].lower() == "rsb":
                print ">>> RSB is currrenly not supported :("
                self.run = False
                sys.exit(1)
            else:
                print ">>> Unknown middleware %s" % datatypes[0]
                self.run = False
                sys.exit(1)
            self.input_sources.append(mw)
            time.sleep(0.1)
            # Gaze Control
            gc = g.GazeController(self.rd, mw)
            self.gaze_controller.append(gc)
            idx += 1
            time.sleep(0.1)

    def arbitrate(self):
        while self.run:
            self.get_latest_targets()
            time.sleep(0.05)
        print ">>> Stopping Arbitration"

    def get_latest_targets(self):
        updates = []
        for target in self.input_sources:
            if target.current_robot_gaze is not None:
                updates.append(target.current_robot_gaze.timestamp)
            else:
                updates.append(None)
        self.derive_order(updates)

    def derive_order(self, _updates):
        idx = -1
        # Now honor priority and latest input
        now = time.time()
        # Default winner is always highest prio
        winner = 0
        for stamp in _updates:
            if stamp is not None:
                if now - stamp <= self.boring:
                    idx += 1
                    winner = idx
                    break
                else:
                    # Too boring advance in prios
                    idx += 1
            else:
                # Next prio, because we don't have any values yet.
                idx += 1
        # Now enable the correct gaze controller
        idx = 0
        for gz in self.gaze_controller:
            if idx == winner:
                gz.acquire_prio = True
                print ">>> Winning input is %s" % self.input_sources[winner].inscope
            else:
                gz.acquire_prio = False
            idx += 1