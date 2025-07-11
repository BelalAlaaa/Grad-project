#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>
#include "KalmanSensor.h"

const char* ssid = "WE_8ABD0C";
const char* password = "m3605330";

// Main endpoint to send sensor data
const char* serverUrl = "http://192.168.1.28:5000/receive";

// Optional: Control endpoint
const char* commandUrl = "http://192.168.1.28:5000/command";

const char* espUsername = "Ventilation_System_ESP";
const char* espPassword = "password";

extern bool systemActive;

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void parseServerResponse(const String& response) {
  Serial.println("Server Response: " + response);
  if (response.indexOf("\"action\":\"VENTILATION_System_on\"") >= 0) {
    systemActive = true;
  } else if (response.indexOf("\"action\":\"VENTILATION_System_off\"") >= 0) {
    systemActive = false;
  }
}

void checkServerCommand() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(commandUrl);

  String authHeader = "Basic " + base64::encode(String(espUsername) + ":" + espPassword);
  http.addHeader("Authorization", authHeader);

  int httpResponseCode = http.GET();
  if (httpResponseCode > 0) {
    String response = http.getString();
    parseServerResponse(response);
  } else {
    Serial.print("Failed to get server command: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void sendSensorData(int sensorID, KalmanSensor& sensor, float rawSoil = 0, float filteredSoil = 0) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(serverUrl);

  String authHeader = "Basic " + base64::encode(String(espUsername) + ":" + espPassword);
  http.addHeader("Authorization", authHeader);
  http.addHeader("Content-Type", "application/json");

  int sensorUniqueID = 0;
  String sensorName;
  float rawValue = 0;
  float filteredValue = 0;

  switch (sensorID) {
    case 0: // T1
      sensorUniqueID = 2001;
      sensorName = "T1";
      rawValue = sensor.getRawTemperature();
      filteredValue = sensor.getFilteredTemperature();
      break;
    case 1: // H1
      sensorUniqueID = 2003;
      sensorName = "H1";
      rawValue = sensor.getRawHumidity();
      filteredValue = sensor.getFilteredHumidity();
      break;
    case 2: // Soil Moisture (S1)
      sensorUniqueID = 2005;
      sensorName = "S1";
      rawValue = rawSoil;
      filteredValue = filteredSoil;
      break;
    case 3: // T2
      sensorUniqueID = 2002;
      sensorName = "T2";
      rawValue = sensor.getRawTemperature();
      filteredValue = sensor.getFilteredTemperature();
      break;
    case 4: // H2
      sensorUniqueID = 2004;
      sensorName = "H2";
      rawValue = sensor.getRawHumidity();
      filteredValue = sensor.getFilteredHumidity();
      break;
    default:
      Serial.println("Invalid sensor ID");
      return;
  }

  String actuatorState = systemActive ? "on" : "off";

  String payload = "{"
    "\"DeviceID\":2000,"
    "\"SensorID\":" + String(sensorUniqueID) + ","
    "\"DeviceName\":\"Ventilation_System_ESP\","
    "\"SensorReadingValue\":" + String(rawValue, 2) + ","
    "\"FilteredValue\":" + String(filteredValue, 2) + ","
    "\"ActuatorID\":2006,"
    "\"ActuatorState\":\"" + actuatorState + "\""
  "}";

  Serial.println("Sending payload: " + payload);

  int httpResponseCode = http.POST(payload);
  if (httpResponseCode > 0) {
    String response = http.getString();
    parseServerResponse(response);  // Optional action reply
  } else {
    Serial.print("Error sending data: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

#endif
