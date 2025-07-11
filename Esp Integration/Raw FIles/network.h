#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>  // Ensure you have the Base64 library installed

const char* ssid = "WE_8ABD0C";
const char* password = "m3605330";
const char* serverUrl = "http://192.168.1.28:5000/receive";
const char* espUsername = "Ventilation_System_ESP";
const char* espPassword = "password";


void setup() {
  Serial.begin(115200);

  // Initialize the built-in LED pin


  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);

    // Add Basic Authentication headers
    String authHeader = "Basic " + base64::encode(String(espUsername) + ":" + espPassword);
    http.addHeader("Authorization", authHeader);
    http.addHeader("Content-Type", "application/json");

    
    float temperature = 30;
    float kalman_temperature = 30;
    // JSON payload with kalman_temperature added
    String payload = "{"
                     "\"DeviceID\":1,"
                     "\"DeviceName\":\"" + String(espUsername) + "\","
                     "\"temperature\":" + String(temperature, 2) + ","
                     "\"kalman_temperature\":" + String(kalman_temperature, 2) +
                     "}";
    

    Serial.println("Sending payload: " + payload);

    // Send HTTP POST request
    int httpResponseCode = http.POST(payload);
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Response from server: " + response);

      // Parse server response
      if (response.indexOf("\"DeviceName\":\"" + String(espUsername) + "\"") != -1) {
        if (response.indexOf("\"action\":\"On\"") != -1) {
          Serial.println("ventilation system is on by server command");
          // add function for action
        }
      } else {
        Serial.println("Response not for this ESP");
      }
    } else {
      Serial.println("Error sending data");
    }
    http.end();
  }
  delay(5000);   // Send data every 5 seconds
}