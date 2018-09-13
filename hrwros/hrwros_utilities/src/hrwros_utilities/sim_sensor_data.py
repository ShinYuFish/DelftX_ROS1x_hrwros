import random
def distSensorData(sensor_type, min_range, max_range):
  if (sensor_type != 0):
    print('Sensor type not supported!')
    return -1
  else:
    if(random.uniform(0.0, 1.0) < 0.95):
      return max_range
    else:
      return random.uniform(min_range, max_range)
