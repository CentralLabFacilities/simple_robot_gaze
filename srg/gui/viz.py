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

# STD
import time
import random
import threading

# PyQT
from PyQt4 import QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import pyqtSlot

# SELF
from srg.middleware import ros as r


class Viz(QtGui.QWidget):
    def __init__(self, _input_source, _gaze_controller, _arbitration):
        super(Viz, self).__init__()

        self.arbitration  = _arbitration
        self.input_sources   = _input_source
        self.gaze_controller = _gaze_controller

        self.tc = r.ToggleConnector()
        self.ispaused = False
        self.run = True

        self.font = QtGui.QFont()
        self.font.setPointSize(10)
        self.font.setBold(True)
        self.font.setWeight(75)

        self.layout = QtGui.QVBoxLayout(self)
        self.ccs_label = QtGui.QLabel("Current Control Stimulus :")
        self.ccs_label.setFont(self.font)
        self.layout.addWidget(self.ccs_label)

        self.current_targets = {}
        self.current_activity = {}
        self.info_labels = {}
        for gc in self.gaze_controller:
            name = gc.mw.inscope
            self.info_labels[name] = QtGui.QLabel(name)
            self.current_activity[name] = QtGui.QProgressBar()
            self.current_activity[name].setMaximum(100)
            self.current_activity[name].setMinimum(1)
            self.current_activity[name].setValue(random.randint(1, 100))
        for label in self.info_labels:
            self.layout.addWidget(self.info_labels[label])
            self.layout.addWidget(self.current_activity[label])

        self.pause_button = QPushButton('Pause Auto Gaze', self)
        self.pause_button.clicked.connect(self.pause)
        self.layout.addWidget(self.pause_button)

        # self.quit_button = QPushButton('Close Simple Robot Gaze', self)
        # self.quit_button.clicked.connect(self.exit_srg)
        # self.layout.addWidget(self.quit_button)

        self.init_ui()

    def start_viz(self):
        gt = threading.Thread(target=self.get_control_data)
        gt.start()

    def get_control_data(self):
        while self.run:
            if self.arbitration.winner is not None:
                self.ccs_label.setText("Current Controlling Stimulus: "+self.input_sources[self.arbitration.winner].inscope)
                for gc in self.gaze_controller:
                    self.current_targets[gc.mw.inscope] = [ int(gc.mw.current_robot_gaze.pan), int(gc.mw.current_robot_gaze.tilt) ]
                for data in self.current_targets.keys():
                    self.info_labels[data].setText("Current Targets @ "+data+" >>> "+str(self.current_targets[data]))
            # Update GUI every 100ms
            time.sleep(0.1)

    def pause(self):
            if self.ispaused is False:
                self.tc.pause()
                self.ispaused = True
                self.pause_button.setText("Resume Auto Gaze")
            else:
                self.tc.resume()
                self.ispaused = False
                self.pause_button.setText("Pause Auto Gaze")

    def exit_srg(self):
        self.arbitration.request_stop()

    def init_ui(self):
        self.setGeometry(100, 100, 640, 100)
        self.setWindowTitle(":: Florian's Simple Robot Gaze :: [10 Hz GUI Update Rate]")
        self.show()
