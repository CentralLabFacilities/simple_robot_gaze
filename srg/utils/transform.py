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

import sys

class AffineTransform:
    """
    Derives the transformation between screen
    coordinates in pixels and joint axis angles in degree.
    """
    def __init__(self, _name):
        print">>> Initializing Affine Transform for: %s" % _name
        # Target ---> The ones you want to map to
        self.target0 = [1.0, 1.0]
        self.target1 = [1.0, 1.0]
        self.target2 = [1.0, 1.0]
        self.target3 = [1.0, 1.0]

        # Origin ---> The ones that are mapped to [target0, target1, target2, target3]
        self.origin0 = [1.0, 1.0]
        self.origin1 = [1.0, 1.0]
        self.origin2 = [1.0, 1.0]
        self.origin3 = [1.0, 1.0]

        # Divider
        self.divider = 1.0

        # Calculated and mapped Coordinates
        mappedCoords = [1.0, 1.0]

        # Affine transformation coefficients
        self.An = 1.0
        self.Bn = 1.0
        self.Cn = 1.0
        self.Dn = 1.0
        self.En = 1.0
        self.Fn = 1.0

        # Test coord
        self.test = [1.0, 1.0]

    def set_coords(self, _x, _y, _fov_h, _fov_v):

        print">>> Deriving mapping: %.fx%.f (pixels) to %.fx%.f (fov)" % (_x, _y, _fov_h, _fov_v)

        fov_h = _fov_h
        fov_v = _fov_v

        # Upper left corner
        self.target0[0] = -fov_h/2.0
        self.target0[1] = fov_v/2.0

        # Lower left corner
        self.target1[0] = -fov_h/2.0
        self.target1[1] = -fov_v/2.0

        # Upper right corner
        self.target2[0] = fov_h/2.0
        self.target2[1] = fov_v/2.0

        # Lower right corner
        self.target3[0] = fov_h/2.0
        self.target3[1] = -fov_v/2.0

        # This is the origin system, is mapped to [t0,t1,t2,t3]
        # Upper left corner
        self.origin0[0] = 0.0
        self.origin0[1] = 0.0

        # Lower left corner
        self.origin1[0] = 0.0
        self.origin1[1] = _y

         # Upper right corner
        self.origin2[0] = _x
        self.origin2[1] = 0.0

         # Lower right corner
        self.origin3[0] = _x
        self.origin3[1] = _y

        # And finally the test coordinate
        self.test[0] = 512.0
        self.test[1] = 384.0

    def calculate_divider(self):
        result = ((self.origin0[0] - self.origin2[0]) * (self.origin1[1] - self.origin2[1])) - \
                 ((self.origin1[0] - self.origin2[0]) * (self.origin0[1] - self.origin2[1]))

        if result == 0.0:
            print(">> Divider is ZERO - Check your Coordinates?")
            sys.exit(1)
        else:
            self.divider = result
            # print(">> Divider " + str(self.divider))
            self.calculateAn()
            self.calculateBn()
            self.calculateCn()
            self.calculateDn()
            self.calculateEn()
            self.calculateFn()

        return result

    def calculateAn(self):
        result = ((self.target0[0] - self.target2[0]) * (self.origin1[1] - self.origin2[1])) - \
                 ((self.target1[0] - self.target2[0]) * (self.origin0[1] - self.origin2[1]))
        self.An = result
        # print(">> An " + str(self.An))
        return result

    def calculateBn(self):
        result = ((self.origin0[0] - self.origin2[0]) * (self.target1[0] - self.target2[0])) - \
                 ((self.target0[0] - self.target2[0]) * (self.origin1[0] - self.origin2[0]))
        self.Bn = result
        # print(">> Bn " + str(self.Bn))
        return result

    def calculateCn(self):
        result = (self.origin2[0] * self.target1[0] - self.origin1[0] * self.target2[0]) * self.origin0[1] + \
                 (self.origin0[0] * self.target2[0] - self.origin2[0] * self.target0[0]) * self.origin1[1] + \
                 (self.origin1[0] * self.target0[0] - self.origin0[0] * self.target1[0]) * self.origin2[1]
        self.Cn = result
        # print(">> Cn " + str(self.Cn))
        return result

    def calculateDn(self):
        result = ((self.target0[1] - self.target2[1]) * (self.origin1[1] - self.origin2[1])) - \
                 ((self.target1[1] - self.target2[1]) * (self.origin0[1] - self.origin2[1]))
        self.Dn = result
        # print(">> Dn " + str(self.Dn))
        return result

    def calculateEn(self):
        result = ((self.origin0[0] - self.origin2[0]) * (self.target1[1] - self.target2[1])) - \
                 ((self.target0[1] - self.target2[1]) * (self.origin1[0] - self.origin2[0]))
        self.En = result
        # print(">> En " + str(self.En))
        return result

    def calculateFn(self):
        result = (self.origin2[0] * self.target1[1] - self.origin1[0] * self.target2[1]) * self.origin0[1] + \
                 (self.origin0[0] * self.target2[1] - self.origin2[0] * self.target0[1]) * self.origin1[1] + \
                 (self.origin1[0] * self.target0[1] - self.origin0[0] * self.target1[1]) * self.origin2[1]
        self.Fn = result
        # print(">> Fn " + str(self.Fn))
        return result

    def derive_mapping_coords(self, point):
        # r->x = ((matrixPtr->An * ad->x) + (matrixPtr->Bn * ad->y) + matrixPtr->Cn) / matrixPtr->Divider
        # r->y = ((matrixPtr->Dn * ad->x) + (matrixPtr->En * ad->y) + matrixPtr->Fn) / matrixPtr->Divider
        if self.divider != 0.0:
            x = ((self.An * point[0]) + (self.Bn * point[1]) + self.Cn) / self.divider
            y = ((self.Dn * point[0]) + (self.En * point[1]) + self.Fn) / self.divider
            result = [x, y]
            # print ">> Current Coordinate (Face):", point
            # print ">> x-pixels were mapped to angle: %s \n>> y-pixels were mapped to angle: %s" % (str(x), str(y))
            return result
        else:
            return None