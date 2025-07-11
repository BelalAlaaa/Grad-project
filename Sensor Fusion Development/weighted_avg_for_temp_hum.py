import pandas as pd

# Function for weighted average fusion
def weighted_average_fusion(sensor_data, weights):
    n = len(sensor_data)
    fused_value = []
    subreading = [0, 0, 0]
    subweight = [0, 0, 0]
    for i in range(int(n / 3)):
        for j in range(3):
            subreading[j] = sensor_data[3 * i + j]
            subweight[j] = weights[3 * i + j]
        result = subreading[0] * subweight[0] + subreading[1] * subweight[1] + subreading[2] * subweight[2]
        fused_value.append(result)
    return fused_value

# Read data from CSV file
file_path = "fusiondata.csv"
data = pd.read_csv(file_path)

# Temperature Processing
# names of the columns of temp in csv
temp_1 = data["Filtered Temp 1"]
temp_2 = data["Filtered Temp 2"]
temp_3 = data["Filtered Temp 3"]

# Combine the values in the specified order
sensor_readings_temp = []
for i in range(len(temp_1)):
    sensor_readings_temp.append(temp_1[i])
    sensor_readings_temp.append(temp_2[i])
    sensor_readings_temp.append(temp_3[i])

# Filter out temperature readings outside 10-30 range
filtered_temp = [reading for reading in sensor_readings_temp if 10 <= reading <= 50]

# Assign temperature weights
weights = data.loc[0, ["W1", "W2", "W3"]].tolist()
sensor_weights_temp = weights * (len(filtered_temp) // len(weights))

# Fuse temperature readings
if len(filtered_temp) != len(sensor_weights_temp):
    print("Error: Temp readings and weights mismatch.")
    fused_temp = []
else:
    fused_temp = weighted_average_fusion(filtered_temp, sensor_weights_temp)
    print("Fused Temperature Readings:", fused_temp)

# Humidity Processing
hum_1 = data["Filtered Hum 1"]
hum_2 = data["Filtered Hum 2"]
hum_3 = data["Filtered Hum 3"]
sensor_readings_hum = []
for i in range(len(hum_1)):
    sensor_readings_hum.append(hum_1[i])
    sensor_readings_hum.append(hum_2[i])
    sensor_readings_hum.append(hum_3[i])

# Filter humidity values to range 10–80%
filtered_hum = [reading for reading in sensor_readings_hum if 10 <= reading <= 100]

# Assign humidity weights
sensor_weights_hum = weights * (len(filtered_hum) // len(weights))

# Fuse humidity readings
if len(filtered_hum) != len(sensor_weights_hum):
    print("Error: Humidity readings and weights mismatch.")
    fused_hum = []
else:
    fused_hum = weighted_average_fusion(filtered_hum, sensor_weights_hum)
    print("Fused Humidity Readings:", fused_hum)
