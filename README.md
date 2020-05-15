# pyrouter
Reduce GPS data error with Kalman Filter and sensor fusion.

For obtain GPS data I used Android application Phyphox https://phyphox.org/

Requeriments:
numpy
scipy
pandas
matplotlib
pykml

Usage:
python pyrouter.py kml_file route_file accel_file

  kml_file    KML map *.kml
  route_file  GPS route map *.csv
  accel_file  Acceleration route data *.csv
