# Import standard libraries
import serial
import time

# Begin serial communication with gps
gps = serial.Serial(port='/dev/ttyUSB0', baudrate=4800, timeout=0.1)

# Function to convert coordinates to degrees decimal
def convert_to_degrees_decimal(lat, lat_orientation, lon, lon_orientation, alt):
  # Get latitude in degrees decimal
  lat_temp = float(lat) / 100.0
  lat_degrees = int(lat_temp)
  lat_decimal = (lat_temp - lat_degrees) * 100 / 60
  lat = lat_degrees + lat_decimal

  # Get longitude in degrees decimal
  lon_temp = float(lon) / 100.0
  lon_degrees = int(lat_temp)
  lon_decimal = (lon_temp - lon_degrees) * 100 / 60
  lon = lon_degrees + lon_decimal

  # Set altitude and orientation
  alt = float(alt)

  if lat_orientation.lower() in "s":
    lat = -lat

  if lon_orientation.lower() in "w":
    lon = -lon

  return lat, lon, alt


# Running main loop
while True:
  time.sleep(1)  # wait for 1 sec to update GPS value
  line = gps.readline()  # read serial comm line
  splits = str(line).strip("b'").split(',')
  
  if splits[0] == "$GPGGA":
    latitude = splits[2]
    lat_direc = splits[3]
    longitude = splits[4]
    lon_direc = splits[5]
    altitude = splits[9]

    lat, lon, alt = convert_to_degrees_decimal(latitude, lat_direc, longitude, lon_direc, altitude)
    print(lat, lon, alt)
