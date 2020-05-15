"""
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import math
import numpy as np

a = 6378137
b = 6356752.3142
f = (a-b) / a
e_sq = f * (2-f)


class GeoUtils:
    def geodetic_to_ECEF(self, lat, lon, h):
        phi = math.radians(lat)
        lamb = math.radians(lon)
        s = math.sin(phi)
        N = a / math.sqrt(1 - e_sq*s*s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        X = (N+h) * cos_phi * cos_lambda
        Y = (N+h) * cos_phi * sin_lambda
        Z = ((1-e_sq) * N + h) * sin_phi

        return X, Y, Z

    def ECEF_to_geodetic(self, X, Y, Z):
        r = math.sqrt(X**2 + Y**2)
        eps = e_sq / (1.0 - e_sq)
        F = 54 * (b**2) * (Z**2)
        G = (r**2) + (1 - e_sq) * (Z**2) - e_sq * (a**2 - b**2)
        c = (e_sq**2) * F * (r**2) / (G**3)
        s = np.cbrt(1 + c + math.sqrt(c**2 + 2*c))
        P = F / (3 * ((s + 1/s + 1)**2) * (G**2))
        Q = math.sqrt(1 + 2 * (e_sq**2) * P)
        tmp = (1 + 1/Q) * (a**2) / 2 - P * (1-e_sq) * (Z**2) / (Q * (1+Q)) - P * (r**2) / 2
        r0 = (-P * e_sq * r) / (1+Q) + math.sqrt(tmp)
        U = math.sqrt((r - e_sq*r0)**2 + Z**2)
        V = math.sqrt((r - e_sq*r0)**2 + (1-e_sq) * (Z**2))
        z0 = ((b**2) * Z) / (a*V)
        h = U * (1 - (b**2) / (a*V))
        phi = math.degrees(math.atan((Z + eps*z0) / r))
        lamb = math.degrees(math.atan2(Y, X))

        return phi, lamb, h

    def ECEF_to_ENU(self, X, Y, Z, lat0, lon0, h0):
        phi0 = math.radians(lat0)
        lamb0 = math.radians(lon0)
        sin_lambda0 = math.sin(lamb0)
        cos_lambda0 = math.cos(lamb0)
        sin_phi0 = math.sin(phi0)
        cos_phi0 = math.cos(phi0)

        X0, Y0, Z0 = self.geodetic_to_ECEF(lat0, lon0, h0)
        Xd = X - X0
        Yd = Y - Y0
        Zd = Z - Z0
        x = -sin_lambda0*Xd + cos_lambda0*Yd
        y = -sin_phi0*cos_lambda0*Xd - sin_phi0*sin_lambda0*Yd + cos_phi0*Zd
        z = cos_phi0*cos_lambda0*Xd + cos_phi0*sin_lambda0*Yd + sin_phi0*Zd

        return x, y, z

    def ENU_to_ECEF(self, x, y, z, lat0, lon0, h0):
        phi = math.radians(lat0)
        lamb = math.radians(lon0)
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        X0, Y0, Z0 = self.geodetic_to_ECEF(lat0, lon0, h0)
        Xd = -sin_lambda*x - sin_phi*cos_lambda*y + cos_phi*cos_lambda*z
        Yd = cos_lambda*x - sin_phi*sin_lambda*y + cos_phi*sin_lambda*z
        Zd = cos_phi*y + sin_phi*z
        X = Xd + X0
        Y = Yd + Y0
        Z = Zd + Z0

        return X, Y, Z

    def geodetic_to_ENU(self, lat, lon, h, lat0, lon0, h0):
        X, Y, Z = self.geodetic_to_ECEF(lat, lon, h)
        x, y, z = self.ECEF_to_ENU(X, Y, Z, lat0, lon0, h0)

        return x, y, z

    def ENU_to_geodetic(self, x, y, z, lat0, lon0, h0):
        X, Y, Z = self.ENU_to_ECEF(x, y, z, lat0, lon0, h0)
        phi, lamb, h = self.ECEF_to_geodetic(X, Y, Z)

        return phi, lamb, h
