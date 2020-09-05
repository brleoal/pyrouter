# pyrouter
Reduce GPS data error with Kalman Filter and sensor fusion.

For obtain GPS data I used Android application Phyphox https://phyphox.org/

Requeriments:
- numpy
- scipy
- pandas
- matplotlib
- pykml

Usage:

```sh
$ python pyrouter.py KML_FILE ROUTE_FILE ACCEL_FILE
```

KML_FILE: KML map *.kml

ROUTE_FILE: GPS route map *.csv

ACCEL_FILE: Acceleration route data *.csv
