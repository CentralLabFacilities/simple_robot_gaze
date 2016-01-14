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
import operator
import threading

# RSB Specifics
import rsb
from rstsandbox.geometry.SphericalDirectionFloat_pb2 import SphericalDirectionFloat
from rst.vision.Face_pb2 import Face
from rst.vision.Faces_pb2 import Faces
from rst.geometry.BoundingBox_pb2 import BoundingBox

# HLRC IMPORTS
from hlrc_client import RobotGaze
from hlrc_client import RobotTimestamp


class RSBPauseConnector(threading.Thread):

    def __init__(self, _prefix, _paused, _pause_lock):
        threading.Thread.__init__(self)
        self.run_toggle = True
        self.lock       = _pause_lock
        self.paused     = _paused
        self.is_paused  = False
        self.prefix     = str("/"+_prefix.lower().strip())
        self.setscope   = str(self.prefix+"/robotgaze/set/pause").strip()
        self.outscope   = str(self.prefix+"/robotgaze/get/pause").strip()
        self.toggle_setter   = rsb.createInformer(self.setscope, dataType=bool)
        self.toggle_informer = rsb.createInformer(self.outscope, dataType=bool)
        self.p = True
        self.r = False

    def pause(self):
        self.toggle_setter.publishData(self.p)

    def resume(self):
        self.toggle_setter.publishData(self.r)

    def run(self):
        print ">>> Initializing RSB Pause Publisher to: %s" % self.outscope
        while self.run_toggle is True:
            self.lock.acquire()
            self.toggle_informer.publishData(self.paused.get_paused())
            self.is_paused = self.paused.get_paused()
            self.lock.release()
            time.sleep(0.05)
        self.toggle_informer.deactivate()
        self.toggle_setter.deactivate()
        print ">>> Deactivating RSB Pause Publisher to: %s" % self.outscope


class RSBSetDirectGazeConnector(threading.Thread):

    def __init__(self, _prefix, _rd):
        threading.Thread.__init__(self)
        self.run_toggle = True
        self.robot_driver = _rd
        self.point_x = 0.0
        self.point_y = 0.0
        self.point_z = 0.0
        self.ready = False
        self.last_robot_gaze = None
        self.last_time_stamp = time.time()
        self.prefix = "/"+str(_prefix.lower().strip())
        self.set_gaze = None
        self.setscope = self.prefix+"/robotgaze/set/gaze"
        self.converter = rsb.converter.ProtocolBufferConverter(messageClass=SphericalDirectionFloat)
        rsb.converter.registerGlobalConverter(self.converter)

    def direct_gaze_callback(self, event):
        if event.data:
            try:
                send_time = event.metaData.sendTime
                self.point_x = event.data.azimuth
                self.point_y = event.data.elevation
                point = [self.point_x, self.point_y]
                if point is not None and self.last_time_stamp != send_time:
                    g = RobotGaze()
                    g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
                    g.gaze_timestamp = RobotTimestamp(send_time)
                    g.pan = point[0]
                    g.tilt = point[1]
                    g.roll = 0.0
                    self.last_robot_gaze = g
                    self.robot_driver.robot_controller.set_gaze_target(g, True)
                    self.last_time_stamp = send_time
                    print ">>> Direct Gaze (RSB) set to: %f, %f" % (g.pan, g.tilt)
            except Exception, e:
                print ">>> Error: ", str(e)

    def run(self):
        print ">>> Initializing RSB Direct Gaze Subscriber to: %s" % self.setscope.strip()
        self.set_gaze = rsb.createListener(self.setscope)
        self.set_gaze.addHandler(self.direct_gaze_callback)
        self.ready = True
        while self.run_toggle is True:
            time.sleep(0.05)
        self.set_gaze.deactivate()
        print ">>> Deactivating RSB Direct Gaze Subscriber to: %s" % self.setscope.strip()


class RSBControlConnector(threading.Thread):

    def __init__(self, _prefix, _pause, _lock):
        threading.Thread.__init__(self)
        self.run_toggle = True
        self.ready = False
        self.pause = _pause
        self.lock  = _lock
        self.toggle_listener = None
        self.prefix  = str("/"+_prefix.lower().strip())
        self.inscope = str(self.prefix+"/robotgaze/set/pause").strip()

    def control_callback(self, event):
        if event.data is True:
            self.lock.acquire()
            self.pause.set_pause()
            self.lock.release()
            print ">>> Auto Arbitrate is PAUSED (RSB)"
        else:
            self.lock.acquire()
            self.pause.set_resume()
            self.lock.release()
            print ">>> Auto Arbitrate is RESUMED (RSB)"

    def run(self):
        print ">>> Initializing RSB Pause Subscriber to: %s" % self.inscope.strip()
        self.toggle_listener = rsb.createListener(self.inscope)
        self.toggle_listener.addHandler(self.control_callback)
        self.ready = True
        while self.run_toggle is True:
            time.sleep(0.05)
        self.toggle_listener.deactivate()
        print ">>> Deactivating RSB Pause Subscriber to: %s" % self.inscope.strip()


class RSBDataConnector(threading.Thread):
    """
    The GazeController receives person messages (ROS) and derives
    the nearest person identified. Based on this, the robot's
    joint angle target's are derived using the transformation
    class below
    """
    def __init__(self, _inscope, _transform, _datatype, _mode, _stimulus_timeout, _lock):
        threading.Thread.__init__(self)
        self.lock       = _lock
        self.run_toggle = True
        self.ready    = False
        self.trans    = _transform
        self.inscope  = str(_inscope).lower().strip()
        self.datatype = str(_datatype).lower().strip()
        self.mode     = str(_mode).lower().strip()
        self.stimulus_timeout = float(_stimulus_timeout)
        self.nearest_person_x = 0.0
        self.nearest_person_y = 0.0
        self.nearest_person_z = 0.0
        self.roi_x            = 0.0
        self.roi_y            = 0.0
        self.point_x          = 0.0
        self.point_y          = 0.0
        self.point_z          = 0.0
        self.current_robot_gaze = None
        self.current_robot_gaze_timestamp = None
        self.f  = None
        self.fs = None
        self.b  = None
        self.s  = None

    def faces_callback(self, event):
        self.lock.acquire()
        send_time = event.metaData.sendTime
        idx = -1
        max_distance = {}
        print event.getData()
        for face in event.getData():
            print face
            idx += 1
            max_distance[str(idx)] = face.region.width * face.region.height
        sort = sorted(max_distance.items(), key=operator.itemgetter(1), reverse=True)
        self.nearest_person_x = event.Faces[int(sort[0][0])].region.top_left.x
        self.nearest_person_y = event.Faces[int(sort[0][0])].region.top_left.y
        self.nearest_person_z = event.Faces[int(sort[0][0])].face.region.width * event.Faces[int(sort[0][0])].face.region.height
        point = [self.nearest_person_x, self.nearest_person_y]
        # Derive coordinate mapping
        angles = self.trans.derive_mapping_coords(point)
        if angles is not None:
            g = RobotGaze()
            if self.mode == 'relative':
                g.gaze_type = RobotGaze.GAZETARGET_RELATIVE
            else:
                g.gaze_type = RobotGaze.GAZETARGET_ABSOLUTE
            self.current_robot_gaze_timestamp = send_time
            g.gaze_timestamp = RobotTimestamp(self.current_robot_gaze_timestamp)
            g.pan = angles[0]
            g.tilt = angles[1]
            self.current_robot_gaze = g
            print g
        self.lock.release()
        self.honor_stimulus_timeout()

    def honor_stimulus_timeout(self):
        time.sleep(self.stimulus_timeout)

    def run(self):
        print ">>> Initializing RSB Subscriber to: %s" % self.inscope.strip()
        try:
            if self.datatype == "faces":
                try:
                    self.f  = rsb.converter.ProtocolBufferConverter(messageClass=Face)
                    self.fs = rsb.converter.ProtocolBufferConverter(messageClass=Faces)
                    self.b  = rsb.converter.ProtocolBufferConverter(messageClass=BoundingBox)
                    rsb.converter.registerGlobalConverter(self.f)
                    rsb.converter.registerGlobalConverter(self.fs)
                    rsb.converter.registerGlobalConverter(self.b)
                except Exception, e:
                    print ">>> ERROR %s" % str(e)
                rsb_subscriber = rsb.createListener(self.inscope)
                rsb_subscriber.addHandler(self.faces_callback)
            elif self.datatype == "sphericaldirectionfloat":
                try:
                    self.s = rsb.converter.ProtocolBufferConverter(messageClass=SphericalDirectionFloat)
                    rsb.converter.registerGlobalConverter(self.s)
                except Exception, e:
                    print ">>> ERROR %s" % str(e)
                rsb_subscriber = rsb.createListener(self.inscope)
                print ">>> WARNING: No Handler Implemented..."
            else:
                print ">>> RSB Subscriber DataType not supported %s" % self.datatype.strip()
                return
        except Exception, e:
            print ">>> ERROR %s" % str(e)
            return
        self.ready = True
        while self.run_toggle is True:
            time.sleep(0.05)
        rsb_subscriber.deactivate()
        print ">>> Deactivating ROS Subscriber to: %s" % self.inscope.strip()
