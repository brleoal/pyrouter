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

from kmltocsv import KMLtoCSV
from geoutils import GeoUtils
from kalman import KalmanFilter
import os
import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

KML_PATH = os.path.join("datasets", "googlemaps")
GPS_PATH = os.path.join("datasets", "gpsdata")
ACCEL_PATH = os.path.join("datasets", "acceldata")


def GPStoENU(gps_map, lat0, lon0, h0):
    geo = GeoUtils()
    lst = []
    for i in gps_map.index:
        x, y, z = geo.geodetic_to_ENU(gps_map["Latitude (°)"][i], gps_map["Longitude (°)"][i],
                                      gps_map["Height (m)"][i], lat0, lon0, h0)
        lst.append([x, y, z])

    return pd.DataFrame(lst, columns=['x', 'y', 'z'])


def ENUtoGPS(enu_map, lat0, lon0, h0):
    geo = GeoUtils()
    lst = []
    for i in enu_map.index:
        x, y, z = geo.ENU_to_geodetic(enu_map['x'][i], enu_map['y'][i],
                                      enu_map['z'][i], lat0, lon0, h0)
        lst.append([x, y, z])

    return pd.DataFrame(lst, columns=["Latitude (°)", "Longitude (°)", "Height (m)"])


def parseMap(kml_file):
    k = KMLtoCSV()
    csv_file = k.kml_data_to_csv(kml_file)

    return pd.read_csv(csv_file)


def main(args):
    kmlmap = os.path.join(KML_PATH, args.kml_file)
    routegps = os.path.join(GPS_PATH, args.route_file)
    routeaccel = os.path.join(ACCEL_PATH, args.accel_file)

    route_map = parseMap(kmlmap)
    lat0 = route_map["Latitude (°)"][0]
    lon0 = route_map["Longitude (°)"][0]
    h0 = route_map["Height (m)"][0]

    route_GPS = pd.read_csv(routegps)
    route_ENUGPS = GPStoENU(route_GPS, lat0, lon0, h0)
    altitudes = route_ENUGPS['z']
    route_ENUGPS = route_ENUGPS[['x', 'y']]

    route_Accel = pd.read_csv(routeaccel)[["Acceleration x (m/s^2)", "Acceleration y (m/s^2)"]]

    # IMU+GPS with 1Hz
    f = 1.0
    dt = 1 / f  # Time step between filter steps
    numsteps = len(route_ENUGPS.index)  # Measurements

    sa = 0.7  # Acceleration process noise
    sp = 1.891  # Noise of position measurement

    pos = route_ENUGPS.to_numpy()
    accel = np.transpose(route_Accel.truncate(after=numsteps - 1).to_numpy())
    meas = np.stack((pos[:, 0], pos[:, 1]))

    F = np.array([[1.0, 0.0, dt, 0.0],
                  [0.0, 1.0, 0.0, dt],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    H = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0]])
    x0 = np.transpose(np.array([[0.0, 0.0, 0.0, 0.0]]))
    P = np.diag([1.0, 1.0, 1.0, 1.0])
    B = np.array([[1 / 2.0 * dt**2, 0.0],
                  [0.0, 1 / 2.0 * dt**2],
                  [dt, 0.0],
                  [0.0, dt]])
    Q = np.matmul(B, np.transpose(B)) * sa**2
    R = np.array([[sp**2, 0.0],
                  [0.0, sp**2]])

    kf = KalmanFilter(F, B, H, x0, P, Q, R)

    x = []
    y = []
    for i in range(numsteps):
        kf.predict(u_k=accel[:, i].reshape(B.shape[1], 1))
        kf.update(z_k=meas[:, i].reshape(H.shape[0], 1))
        x.append(float(kf.x_k[0]))
        y.append(float(kf.x_k[1]))

    x = signal.savgol_filter(x, 39, 3)
    y = signal.savgol_filter(y, 39, 3)
    route_ENUKF = pd.DataFrame(data={'x': x, 'y': y})
    route_KF = ENUtoGPS(pd.concat([route_ENUKF, altitudes], axis=1), lat0, lon0, h0)

    ax = route_map.plot(kind="line", x="Longitude (°)", y="Latitude (°)", color='r', label="Route map")
    route_GPS.plot(kind="line", x="Longitude (°)", y="Latitude (°)", color='b', label="GPS", ax=ax)
    route_KF.plot(kind="line", x="Longitude (°)", y="Latitude (°)", color='g', label="Kalman", ax=ax)

    plt.xlabel("Longitude (°)")
    plt.ylabel("Latitude (°)")
    plt.show()


if __name__ == "__main__":
    # execute only if run as a script
    import argparse

    parser = argparse.ArgumentParser(description="Reduce GPS data error with Kalman Filter")
    parser.add_argument("kml_file", help="KML map *.kml")
    parser.add_argument("route_file", help="GPS route map *.csv")
    parser.add_argument("accel_file", help="Acceleration route data *.csv")
    args = parser.parse_args()
    main(args)
