#include <ESP32Servo.h>
#include "KalmanSensor.h"
#include "NetworkManager.h"

// === Pin Definitions ===
#define SERVO_PIN 15
#define RELAY_PIN 19

// === Sensor and Actuator Objects ===
Servo myServo;

KalmanSensor sensor1(nullptr, "Sensor 1");  // Simulated T1, H1
KalmanSensor sensor2(nullptr, "Sensor 2");  // Simulated T2, H2

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
  delay(1000); // Allow time for Serial to start

  randomSeed(analogRead(0)); // Initialize random seed

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

    rawSoil = random(300, 800); // Simulated raw soil moisture
    filteredSoil = rawSoil + random(-20, 20); // Simulated filtered

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

    checkServerCommand(); // Fetch control command

    controlDevices(); // Actuator control logic
  }
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
