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

# PyQT
from PyQt4 import QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import *

# SELF
from srg.middleware import ros as r


class LabelThread(QThread):
    def __init__(self, _targets, _activities):
        QThread.__init__(self)
        self.targets = _targets
        self.bars = _activities

    def __del__(self):
        self.wait()

    def run(self):
        while True:
            fake = "None"
            self.emit(SIGNAL('set_control_data(QString)'), str(fake))
            self.msleep(100)


class BarsThread(QThread):
    def __init__(self, _targets, _activities):
        QThread.__init__(self)
        self.targets = _targets
        self.bars = _activities

    def __del__(self):
        self.wait()

    def run(self):
        while True:
            fake = "fake"
            self.emit(SIGNAL('set_bar_values(QString)'), str(fake))
            self.msleep(100)


class Viz(QtGui.QWidget):
    def __init__(self, _input_source, _gaze_controller, _arbitration):
        super(Viz, self).__init__()

        self.arbitration  = _arbitration
        self.input_sources   = _input_source
        self.gaze_controller = _gaze_controller

        self.tc = r.ToggleConnector()
        self.is_paused  = False
        self.run_toggle = True

        self.font = QtGui.QFont()
        self.font.setPointSize(12)
        self.font.setBold(True)
        self.font.setWeight(75)

        self.layout = QtGui.QVBoxLayout(self)
        self.ccs_label = QtGui.QLabel("Controlling Stimulus >")
        self.ccs_label.setFont(self.font)
        self.layout.addWidget(self.ccs_label)

        self.current_targets = {}
        self.current_activity = {}
        self.info_labels = {}
        for gc in self.gaze_controller:
            name = gc.mw.inscope
            self.info_labels[name] = QtGui.QLabel(name)
            self.current_activity[name] = QtGui.QProgressBar()
            self.current_activity[name].setMaximum(24)
            self.current_activity[name].setMinimum(-24)
        for label in self.info_labels:
            self.layout.addWidget(self.info_labels[label])
            self.layout.addWidget(self.current_activity[label])

        if self.arbitration.winner is not None:
            self.ccs_label.setText("Controlling Stimulus > "+self.input_sources[self.arbitration.winner].inscope)
            for gc in self.gaze_controller:
                if gc.mw.current_robot_gaze is not None:
                    self.current_targets[gc.mw.inscope] = [ int(gc.mw.current_robot_gaze.pan), int(gc.mw.current_robot_gaze.tilt) ]
                else:
                    self.current_targets[gc.mw.inscope] = [ -1, -1 ]
            for name in self.current_targets.keys():
                self.info_labels[name].setText("Targets@"+name+" > "+str(self.current_targets[name]))

        self.pause_button = QPushButton('Pause Simple Robot Gaze', self)
        self.pause_button.clicked.connect(self.pause)
        self.layout.addWidget(self.pause_button)

        self.get_bars_thread = None
        self.get_label_thread = None

        self.init_ui()

    def start_update_threads(self):
        self.get_label_thread = LabelThread(self.current_targets, self.current_activity)
        self.connect(self.get_label_thread, SIGNAL("set_control_data(QString)"), self.set_control_data)
        self.get_label_thread.start()

        self.get_bars_thread = BarsThread(self.current_targets, self.current_activity)
        self.connect(self.get_bars_thread, SIGNAL("set_bar_values(QString)"), self.set_bar_values)
        self.get_bars_thread.start()

    def set_bar_values(self, _values):
            for label in self.info_labels:
                if label in self.current_targets.keys() and label in self.current_activity.keys():
                    try:
                        self.current_activity[label].setValue(self.current_targets[label][0])
                    except Exception, e:
                        pass

    def set_control_data(self, _values):
            if self.arbitration.winner is not None:
                self.ccs_label.setText("Controlling Stimulus > "+self.input_sources[self.arbitration.winner].inscope)
                for gc in self.gaze_controller:
                    try:
                        self.current_targets[gc.mw.inscope] = [ int(gc.mw.current_robot_gaze.pan), int(gc.mw.current_robot_gaze.tilt) ]
                    except Exception, e:
                        pass
                for name in self.current_targets.keys():
                    try:
                        self.info_labels[name].setText("Targets@"+name+" > "+str(self.current_targets[name]))
                    except Exception, e:
                        pass

    def pause(self):
            if self.is_paused is False:
                self.tc.pause()
                self.is_paused = True
                self.pause_button.setText("Resume Simple Robot Gaze")
            else:
                self.tc.resume()
                self.is_paused = False
                self.pause_button.setText("Pause Simple Robot Gaze")

    def exit_srg(self):
        self.arbitration.request_stop()

    def init_ui(self):
        self.setGeometry(100, 100, 640, 100)
        self.setWindowTitle(":: Florian's Simple Robot Gaze :: [Update Rate 10 Hz]")
