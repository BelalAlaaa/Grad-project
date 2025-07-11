#include <Arduino.h>
#include <DHT.h>
#include "SimpleKalmanFilter.h"

#define DHTPIN 4          
#define DHTTYPE DHT11     

DHT dht(DHTPIN, DHTTYPE);

SimpleKalmanFilter tempFilter(4.0, 2, 0.01);
SimpleKalmanFilter humFilter(25.0, 1.0, 0.01);

const int avgWindow = 5;
float tempBuffer[avgWindow], humBuffer[avgWindow];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  dht.begin();
  delay(1000);
}

void loop() {
  delay(2000); 

  float rawTemp = dht.readTemperature();
  float rawHum = dht.readHumidity();

  if (isnan(rawTemp) || isnan(rawHum)) {
    Serial.println("Failed to read DHT11!");
    return;
  }

  // Update buffers
  tempBuffer[bufferIndex] = rawTemp;
  humBuffer[bufferIndex] = rawHum;
  bufferIndex = (bufferIndex + 1) % avgWindow;

  // Compute averages
  float avgTemp = 0, avgHum = 0;
  for (int i = 0; i < avgWindow; i++) {
    avgTemp += tempBuffer[i];
    avgHum += humBuffer[i];
  }
  avgTemp /= avgWindow;
  avgHum /= avgWindow;

  // Apply Kalman filter
  float filteredTemp = tempFilter.updateEstimate(avgTemp);
  float filteredHum = humFilter.updateEstimate(avgHum);

  // Print data for Serial Plotter (one line, space-separated)
  Serial.print(rawTemp);     // Raw temperature
  Serial.print(" ");         
  Serial.print(filteredTemp); // Filtered temperature
  Serial.print(" ");         
  Serial.print(rawHum);      // Raw humidity
  Serial.print(" ");         
  Serial.println(filteredHum); // Filtered humidity (ends line)
}