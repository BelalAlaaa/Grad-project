#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>

// WiFi credentials
const char* ssid = "Belal";
const char* password = "1234567890";

// Flask server endpoint
const char* serverUrl = "http://192.168.23.172:5000/receive";

// Basic Auth credentials
const char* espUsername = "Ventilation_System_ESP";
const char* espPassword = "password";

// Array size
const int numSensors = 5;  // Adjusted to 5 for full use of data arrays

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());

  sendSensorData();  // Initial send
}

void loop() {
  sendSensorData();
  delay(60000);  // Every 60 seconds
}

// Helper to convert array of values to a JSON array string
String buildJSONArray(float values[], int size) {
  String json = "[";
  for (int i = 0; i < size; i++) {
    json += String(values[i], 2);  // 2 decimal places
    if (i < size - 1) json += ",";
  }
  json += "]";
  return json;
}

String buildJSONArray(int values[], int size) {
  String json = "[";
  for (int i = 0; i < size; i++) {
    json += String(values[i]);
    if (i < size - 1) json += ",";
  }
  json += "]";
  return json;
}

void sendSensorData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return;
  }
  // ven system sim
  // Simulated sensor values should use real ones instead
  int sensorIDs[numSensors] = {2001, 2002, 2005, 2003, 2004};

  float temperature[numSensors] =         {30.0, 0.0, 0.0, 31.0, 0};
  float filteredTemperature[numSensors] = {30.0, 0.0, 0.0, 31.0, 0};

  float humidity[numSensors] =            {0.0, 40.0, 0.0, 0.0, 41};
  float filteredHumidity[numSensors] =    {0.0, 40.0, 0.0, 0.0, 41};

  float soilMoisture[numSensors] =        {0.0, 0.0, 38.0, 0.0, 0};
  float filteredSoilMoisture[numSensors] ={0.0, 0.0, 38.0, 0.0, 0};

  String actuatorState = "on";

  // Construct JSON payload
  String jsonPayload = "{";
  jsonPayload += "\"DeviceName\":\"Ventilation_System_ESP\",";
  jsonPayload += "\"DeviceID\":2000,";
  jsonPayload += "\"ActuatorID\":2009,";
  jsonPayload += "\"ActuatorState\":\"" + actuatorState + "\",";
  jsonPayload += "\"SensorID\":" + buildJSONArray(sensorIDs, numSensors) + ",";
  jsonPayload += "\"temperature\":" + buildJSONArray(temperature, numSensors) + ",";
  jsonPayload += "\"filtered_temperature\":" + buildJSONArray(filteredTemperature, numSensors) + ",";
  jsonPayload += "\"humidity\":" + buildJSONArray(humidity, numSensors) + ",";
  jsonPayload += "\"filtered_humidity\":" + buildJSONArray(filteredHumidity, numSensors) + ",";
  jsonPayload += "\"soil_moisture\":" + buildJSONArray(soilMoisture, numSensors) + ",";
  jsonPayload += "\"filtered_soil_moisture\":" + buildJSONArray(filteredSoilMoisture, numSensors);
  jsonPayload += "}";

  Serial.println("Sending payload:");
  Serial.println(jsonPayload);

  HTTPClient http;
  http.begin(serverUrl);

  // Set headers
  String authHeader = "Basic " + base64::encode(String(espUsername) + ":" + espPassword);
  http.addHeader("Authorization", authHeader);
  http.addHeader("Content-Type", "application/json");

  http.setTimeout(10000);

  int httpResponseCode = http.POST(jsonPayload);

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    Serial.println("Response: " + http.getString());
  } else {
    Serial.print("Error sending POST: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}