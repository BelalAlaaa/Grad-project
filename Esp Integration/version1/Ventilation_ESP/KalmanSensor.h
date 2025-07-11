#ifndef KALMAN_SENSOR_H
#define KALMAN_SENSOR_H

#include <DHT.h>
#include <SimpleKalmanFilter.h>

#define BUFFER_SIZE 50

class KalmanSensor {
private:
  DHT* dht;
  const char* name;
  float tempBuffer[BUFFER_SIZE];
  float humBuffer[BUFFER_SIZE];
  int bufferIndex = 0;
  bool bufferFilled = false;

  float currentTempQ = 0.5;
  float currentTempE = 1.0;
  float currentHumQ = 0.5;
  float currentHumE = 1.0;

  SimpleKalmanFilter kalmanTemp;
  SimpleKalmanFilter kalmanHum;

  float lastFilteredTemp = 0;
  float lastFilteredHum = 0;

  float calcMean(float* data, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) sum += data[i];
    return sum / size;
  }

  float calcStdDev(float* data, int size, float mean) {
    float sumSq = 0;
    for (int i = 0; i < size; i++) {
      float diff = data[i] - mean;
      sumSq += diff * diff;
    }
    return sqrt(sumSq / size);
  }

public:
  KalmanSensor(DHT* sensor, const char* sensorName)
    : dht(sensor), name(sensorName),
      kalmanTemp(0.5, 1.0, 0.02),
      kalmanHum(0.5, 1.0, 0.01)
  {
    for (int i = 0; i < BUFFER_SIZE; i++) {
      tempBuffer[i] = 0;
      humBuffer[i] = 0;
    }
  }

  void updateAndPrint() {
    float rawTemp = dht->readTemperature();
    float rawHum = dht->readHumidity();

    if (isnan(rawTemp) || isnan(rawHum)) {
      Serial.print(name); Serial.println(" failed to read.");
      return;
    }

    tempBuffer[bufferIndex] = rawTemp;
    humBuffer[bufferIndex] = rawHum;

    lastFilteredTemp = kalmanTemp.updateEstimate(rawTemp);
    lastFilteredHum = kalmanHum.updateEstimate(rawHum);

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

    if (bufferFilled) {
      float tMean = calcMean(tempBuffer, BUFFER_SIZE);
      float tStd = calcStdDev(tempBuffer, BUFFER_SIZE, tMean);
      float hMean = calcMean(humBuffer, BUFFER_SIZE);
      float hStd = calcStdDev(humBuffer, BUFFER_SIZE, hMean);

      tStd = constrain(tStd, 0.1f, 2.0f);
      hStd = constrain(hStd, 0.2f, 4.0f);

      float tGap = abs(rawTemp - lastFilteredTemp);
      float hGap = abs(rawHum - lastFilteredHum);

      if (tGap > 2.0f) {
        float newQ = tStd * 0.8f;
        float newE = tStd;
        currentTempQ = 0.7f * currentTempQ + 0.3f * newQ;
        currentTempE = 0.7f * currentTempE + 0.3f * newE;
        kalmanTemp = SimpleKalmanFilter(currentTempQ, currentTempE, 0.1);
      }

      if (hGap > 5.0f) {
        float newQ = hStd * 0.8f;
        float newE = hStd;
        currentHumQ = 0.7f * currentHumQ + 0.3f * newQ;
        currentHumE = 0.7f * currentHumE + 0.3f * newE;
        kalmanHum = SimpleKalmanFilter(currentHumQ, currentHumE, 0.1);
      }
    }

    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferIndex == 0) bufferFilled = true;
  }

  float getFilteredTemperature() {
    return lastFilteredTemp;
  }

  float getFilteredHumidity() {
    return lastFilteredHum;
  }
};

#endif
