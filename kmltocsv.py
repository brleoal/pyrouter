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

import os
from pykml import parser


class KMLtoCSV:
    def kml_data_to_csv(self, kml_file):
        kml_str = open(kml_file, "rb").read()
        root = parser.fromstring(kml_str)
        coordinates = root.Document.Placemark.LineString.coordinates
        gpsdata = str(coordinates).replace(" ", "").split()
        reference = ["Longitude (°),Latitude (°),Height (m)"]
        reference.extend([i for i in gpsdata])
        csv_path = os.path.splitext(kml_file)[0] + ".csv"
        with open(csv_path, 'w') as f:
            for r in reference:
                f.write(r + "\n")

        return csv_path
