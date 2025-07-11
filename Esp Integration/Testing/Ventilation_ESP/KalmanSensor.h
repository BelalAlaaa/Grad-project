#ifndef KALMAN_SENSOR_H
#define KALMAN_SENSOR_H

#define BUFFER_SIZE 50

class KalmanSensor {
private:
  const char* name;
  float lastFilteredTemp = 0;
  float lastFilteredHum = 0;

public:
  KalmanSensor(void* unused, const char* sensorName)
    : name(sensorName) {}

  void updateAndPrint() {
    float rawTemp = random(20, 35); // Simulated temperature in °C
    float rawHum = random(30, 70);  // Simulated humidity in %

    lastFilteredTemp = rawTemp + random(-100, 100) / 100.0f; // Slight noise
    lastFilteredHum = rawHum + random(-200, 200) / 100.0f;

    Serial.print(name);
    Serial.print(" | Temp: ");
    Serial.print(rawTemp);
    Serial.print(" °C -> ");
    Serial.print(lastFilteredTemp);
    Serial.print(" °C | Hum: ");
    Serial.print(rawHum);
    Serial.print(" % -> ");
    Serial.print(lastFilteredHum);
    Serial.println(" %");
  }

  float getFilteredTemperature() {
    return lastFilteredTemp;
  }

  float getFilteredHumidity() {
    return lastFilteredHum;
  }

  // Dummy functions to keep sendSensorData interface the same
  float getRawTemperature() {
    return lastFilteredTemp;
  }

  float getRawHumidity() {
    return lastFilteredHum;
  }
};

#endif
