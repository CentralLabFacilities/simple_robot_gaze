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

# SELF IMPORTS
from srg.robot import driver as d
from srg.utils import transform as t
from srg.control import gaze as g


def runner(arguments):
    if len(arguments) != 3:
        print(">>> Usage: simplerobotgaze.py <inscope 'persons_scope'> <outscope 'gaze_target_scope'>\n\n")
        sys.exit(1)

    rd = d.RobotDriver("ROS", sys.argv[2])
    at = t.AffineTransform()
    at.set_coords()
    at.calculate_divider()
    gc = g.GazeController(rd, at, sys.argv[1])
    gc.run_subscriber()

if __name__ == '__main__':
    runner(sys.argv)