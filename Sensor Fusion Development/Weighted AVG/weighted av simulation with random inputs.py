
def weighted_average_fusion(sensor_data, weights):

  fused_value = 0.0
  for reading, weight in zip(sensor_data, weights):
    fused_value += reading * weight

  return fused_value

# Sensor Readings and its Weights
sensor_readings = [23.2, 22.9, 23.5, 24.0]
sensor_weights = [0.4, 0.15, 0.15, 0.3]       # sum to 1

fused_reading = weighted_average_fusion(sensor_readings, sensor_weights)
print("Fused sensor reading:",fused_reading)




