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
import sys
import time
import yaml
import threading

# SELF IMPORTS
from srg.robot import driver as d
from srg.control import gaze as g
from srg.middleware import ros_conn as r
from srg.middleware import rsb_conn as s
from srg.utils import transform as t
from srg.utils.pause import Paused


class Arbitration(threading.Thread):

    def __init__(self, _configfile, _outscope):
        threading.Thread.__init__(self)
        self.lock             = threading.RLock()
        self.pause_lock       = threading.RLock()
        self.paused_instance  = Paused()
        self.cfgfile          = _configfile.strip()
        self.outscope         = _outscope
        self.last_info        = time.time()
        self.transforms       = []
        self.input_sources    = []
        self.gaze_controller  = []
        self.overrides        = []
        self.run_toggle       = True
        self.is_override          = False
        self.loop_speed           = 1.0
        self.direct_gaze_ros      = None
        self.direct_gaze_rsb      = None
        self.pause_info_ros       = None
        self.pause_info_rsb       = None
        self.override_type        = None
        self.boring               = None
        self.config               = None
        self.winner               = None
        self.arbitrate_toggle_ros = None
        self.arbitrate_toggle_rsb = None
        self.rd                   = None
        self.allow_peak_override  = None
        self.rsb_control_enable   = None
        self.prefix               = ""

    def boot_robot_driver(self, _mw):
        self.rd = d.RobotDriver(_mw, self.outscope.strip())

    def configure(self):
        self.read_yaml_config()
        self.configure_middleware()

    def read_yaml_config(self):
        try:
            print "---"
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
        # Start the external control MW Thread
        self.prefix = self.config["scope_topic_prefix"][0]

        self.arbitrate_toggle_ros = r.ROSControlConnector(self.prefix, self.paused_instance, self.pause_lock)
        self.arbitrate_toggle_ros.start()
        self.pause_info_ros = r.ROSPauseConnector(self.prefix, self.paused_instance, self.pause_lock)
        self.pause_info_ros.start()
        self.direct_gaze_ros = r.ROSSetDirectGazeConnector(self.prefix, self.rd)
        self.direct_gaze_ros.start()

        # Is RSB remote control enabled?
        self.rsb_control_enable = int(self.config["enable_rsb_remote_control"][0])
        if self.rsb_control_enable is 1:
            self.arbitrate_toggle_rsb = s.RSBControlConnector(self.prefix, self.paused_instance, self.pause_lock)
            self.arbitrate_toggle_rsb.start()
            self.pause_info_rsb = s.RSBPauseConnector(self.prefix, self.paused_instance, self.pause_lock)
            self.pause_info_rsb.start()
            self.direct_gaze_rsb = s.RSBSetDirectGazeConnector(self.prefix, self.rd)
            self.direct_gaze_rsb.start()
        else:
            self.arbitrate_toggle_rsb = None
            self.pause_info_rsb = None

        # Read config file an extract values
        idx = 0
        peak_override = int(self.config["allow_peak_override"][0])
        self.boring = float(self.config["boring_timeout"][0])

        for item in self.config["priorities"]:
            res        = self.config["resolution"][idx].split("x")
            fov        = self.config["fov"][idx].split("x")
            datatypes  = self.config["datatypes"][idx].split(":")
            modes      = self.config["modes"][idx]
            stimulus_timeout = self.config["stimulus_timeout"][idx]

            # Check if peak_override is "ON" (1)
            if peak_override is 1:
                self.allow_peak_override = peak_override
                allow_override_threshold = self.config["peak_overrides"][idx]
                self.overrides.append(allow_override_threshold)

            # Configure Affine Transformations
            at = t.AffineTransform(str(item))
            at.set_coords(float(res[0]), float(res[1]), float(fov[0]), float(fov[1]))
            at.calculate_divider()
            self.transforms.append(at)

            # Configure Middleware Adapters
            if datatypes[0].lower() == "ros":
                mw = r.ROSDataConnector(str(item), at, datatypes[1], modes, stimulus_timeout, self.lock)
            elif datatypes[0].lower() == "rsb":
                mw = s.RSBDataConnector(str(item), at, datatypes[1], modes, stimulus_timeout, self.lock)
            else:
                print ">>> Unknown middleware or type %s" % datatypes[0]
                self.run_toggle = False
                sys.exit(1)
            self.input_sources.append(mw)

            # Configure Gaze Controllers
            gc = g.GazeController(self.rd, mw, self.lock)
            self.gaze_controller.append(gc)
            idx += 1

        # RUN EVERYTHING!
        for i_s in self.input_sources:
            i_s.start()
        for g_c in self.gaze_controller:
            g_c.start()

    def request_stop(self):
        for connection in self.input_sources:
            connection.run_toggle = False

        for gazecontrol in self.gaze_controller:
            gazecontrol.run_toggle = False

        self.arbitrate_toggle_ros.run_toggle = False
        self.pause_info_ros.run_toggle = False
        self.direct_gaze_ros.run_toggle = False

        if self.arbitrate_toggle_rsb is not None:
            self.arbitrate_toggle_rsb.run_toggle = False
            self.pause_info_rsb.run_toggle = False
            self.direct_gaze_rsb.run_toggle = False

        time.sleep(0.05)

        self.run_toggle = False

    def get_latest_targets(self):
        updates = []
        stimulus_timeouts = []
        current_gaze_values = []
        for target in self.input_sources:
            if target.current_robot_gaze is not None:
                updates.append(target.current_robot_gaze_timestamp)
                stimulus_timeouts.append(target.stimulus_timeout)
                current_gaze_values.append(target)
            else:
                updates.append(None)
                stimulus_timeouts.append(target.stimulus_timeout)
                current_gaze_values.append(None)
        self.derive_order_and_set_winner(updates, stimulus_timeouts, current_gaze_values)

    def derive_order_and_set_winner(self, _updates, _stimulus_timeouts, _current_gaze_values):
        now = time.time()
        self.is_override = False
        self.override_type = None
        winner = 0
        idx = -1
        p = -1
        n = -1
        if len(_current_gaze_values) != len(_stimulus_timeouts) or len(_current_gaze_values) != len(_updates):
            print ">>> Waiting for data..."
            return
        # Now honor priority and latest input
        if self.allow_peak_override is not None:
            if len(_current_gaze_values) != len(self.overrides) or len(_current_gaze_values) != len(_stimulus_timeouts):
                print ">>> Waiting for data in override mode..."
                return
            for stamp_override in _updates:
                p += 1
                # TODO: Reimplement this using a metric and RSB support.
                if stamp_override is not None:
                    if _current_gaze_values[p].datatype.lower() == "people":
                        if int(_current_gaze_values[p].nearest_person_z) >= int(self.overrides[p]) and now - stamp_override <= _stimulus_timeouts[p] + self.boring:
                            print ">>> Override %s" % _current_gaze_values[p].datatype.lower()
                            winner = p
                            self.is_override = True
                            self.override_type = self.input_sources[p].inscope
                            break
                    if _current_gaze_values[p].datatype.lower() == "pointstamped":
                        if int(_current_gaze_values[p].point_z) <= int(self.overrides[p]) and now - stamp_override <= _stimulus_timeouts[p] + self.boring:
                            print ">>> Override %s" % _current_gaze_values[p].datatype.lower()
                            winner = p
                            self.is_override = True
                            self.override_type = self.input_sources[p].inscope
                            break
        if self.is_override is False:
            for stamp in _updates:
                n += 1
                if stamp is not None:
                    if now - stamp <= _stimulus_timeouts[n] + self.boring:
                        idx += 1
                        winner = idx
                        break
                    else:
                        # Too boring advance in priorities
                        idx += 1
                else:
                    # Next prio, because we don't have any solid values yet.
                    idx += 1

        # Now enable the correct gaze controller, if we dont have anything, always
        # prioritize first input (0)
        idx = 0
        for gz in self.gaze_controller:
            if idx == winner:
                gz.acquire_prio = True
                now = time.time()
                self.winner = winner
                if now - self.last_info >= 1.0:
                    print ">>> Winning input is %s" % self.input_sources[winner].inscope
                    self.last_info = time.time()
            else:
                gz.acquire_prio = False
            idx += 1

    def run(self):
        loop_count = 0
        init_time = time.time()
        while self.run_toggle:
            tick = time.time()
            then = time.time()

            self.pause_lock.acquire()
            is_paused = self.paused_instance.get_paused()
            self.pause_lock.release()

            if is_paused is False:
                self.lock.acquire()
                self.get_latest_targets()
                self.lock.release()
            else:
                for gz in self.gaze_controller:
                    gz.acquire_prio = False

            now = time.time()
            if tick - init_time >= 1.0:
                self.loop_speed = loop_count
                loop_count = 0
                init_time = time.time()
            # Running with maximum frequency of 50 Hz
            hz = 0.02-(now-then)

            if hz > 0:
                time.sleep(hz)

            loop_count += 1
        print ">>> Stopping Arbitration"