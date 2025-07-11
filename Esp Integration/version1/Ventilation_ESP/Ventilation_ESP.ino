#include <DHT.h>
#include <ESP32Servo.h>
#include "KalmanSensor.h"
#include "NetworkManager.h"
#include <SimpleKalmanFilter.h>

// === Pin Definitions ===
#define DHTPIN1 5
#define DHTPIN2 18
#define DHTTYPE DHT11
#define SERVO_PIN 15
#define RELAY_PIN 19
#define SOIL_PIN 34

// === Sensor and Actuator Objects ===
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
Servo myServo;

KalmanSensor sensor1(&dht1, "Sensor 1");  // T1, H1
KalmanSensor sensor2(&dht2, "Sensor 2");  // T2, H2

SimpleKalmanFilter kalmanSoil(0.5, 1.0, 0.01);

// === Timing Variables ===
unsigned long previousSensorMillis = 0;
unsigned long previousNetworkMillis = 0;
const unsigned long sensorInterval = 10000;  // 10 seconds
const unsigned long networkInterval = 60000; // 60 seconds

// === Globals ===
float rawSoil = 0;
float filteredSoil = 0;
bool systemActive = false; // Shared with NetworkManager.h

void setup() {
  Serial.begin(115200);

  dht1.begin();
  dht2.begin();

  myServo.attach(SERVO_PIN);
  myServo.write(90); // shutters closed

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // fan OFF

  setupWiFi(); // Connect to WiFi
}

void loop() {
  unsigned long currentMillis = millis();

  // === Sensor Reading Every 10s ===
  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;

    sensor1.updateAndPrint();
    sensor2.updateAndPrint();

    rawSoil = readSoilMoisture();
    filteredSoil = kalmanSoil.updateEstimate((float)rawSoil);

    Serial.print("Soil Moisture: ");
    Serial.print(rawSoil);
    Serial.print(" -> ");
    Serial.println(filteredSoil);
  }

  // === Network Communication + Command Polling Every 60s ===
  if (currentMillis - previousNetworkMillis >= networkInterval) {
    previousNetworkMillis = currentMillis;

    sendSensorData(0, sensor1, rawSoil, filteredSoil); // T1
    sendSensorData(1, sensor1, rawSoil, filteredSoil); // H1
    sendSensorData(2, sensor1, rawSoil, filteredSoil); // S1
    sendSensorData(3, sensor2, rawSoil, filteredSoil); // T2
    sendSensorData(4, sensor2, rawSoil, filteredSoil); // H2

    checkServerCommand(); // NEW: fetch control command
      // === Actuator Control Logic ===
    controlDevices();
  }


}

// === Soil Moisture Reading ===
int readSoilMoisture() {
  return analogRead(SOIL_PIN);
}

// === Actuator Control Logic ===
void controlDevices() {
  static bool lastState = false;

  if (systemActive && !lastState) {
    Serial.println("[ACTION] Turning ON system from server command");
    myServo.write(0);              // Open shutters
    digitalWrite(RELAY_PIN, LOW); // Turn fan ON
    lastState = true;
  } else if (!systemActive && lastState) {
    Serial.println("[ACTION] Turning OFF system from server command");
    myServo.write(90);             // Close shutters
    digitalWrite(RELAY_PIN, HIGH);// Turn fan OFF
    lastState = false;
  }
}
