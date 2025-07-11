#include <DHT.h>
#include <ESP32Servo.h>

// Pin definitions
#define dataPin1 5     // DHT11 Sensor 1
#define dataPin2 18    // DHT11 Sensor 2
#define servoPin 15    // shutter control
#define relayPin 19    // Relay control
#define soilPin 34     // Soil moisture sensor (analog input)

#define DHTTYPE DHT11

DHT dht1(dataPin1, DHTTYPE);
DHT dht2(dataPin2, DHTTYPE);

Servo myServo;

bool servoState = false;
bool relayState = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting sensors...");

  dht1.begin();
  dht2.begin();

  myServo.attach(servoPin);
  myServo.write(0); // Closed position

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Relay OFF (active LOW)

  delay(2000); // Let sensors stabilize
}

void loop() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 10000 || previousMillis == 0) {
    int secondsElapsed = currentMillis / 1000;
    takeReadings(secondsElapsed / 60.0);
    previousMillis = currentMillis;
  }
}

void takeReadings(float minuteMark) {
  Serial.print("\nAfter ");
  Serial.print(minuteMark, 1);
  Serial.println(" minute(s):");

  float t1 = readAndPrintDHT(dht1, 1);
  float t2 = readAndPrintDHT(dht2, 2);
  int soilValue = readSoilMoisture();

  controlDevices(t1, t2);
}

float readAndPrintDHT(DHT &sensor, int sensorNumber) {
  float t = sensor.readTemperature();
  float h = sensor.readHumidity();

  if (isnan(t) || isnan(h)) {
    Serial.print("Sensor ");
    Serial.print(sensorNumber);
    Serial.println(": Failed to read data!");
    return -1;
  } else {
    Serial.print("Sensor ");
    Serial.print(sensorNumber);
    Serial.print(": Temp = ");
    Serial.print(t);
    Serial.print(" °C, Humidity = ");
    Serial.print(h);
    Serial.println("%");
    return t;
  }
}

int readSoilMoisture() {
  int value = analogRead(soilPin);  // Read raw analog value
  Serial.print("Soil Moisture Sensor: ");
  Serial.print(value);
  Serial.println(" (0=dry, 4095=wet)");
  return value;
}

void controlDevices(float t1, float t2) {
  const float TEMP_THRESHOLD = 24.5;

  if (t1 != -1 && t2 != -1 && t1 > TEMP_THRESHOLD && t2 > TEMP_THRESHOLD) {
    if (!servoState) {
      myServo.write(90);
      servoState = true;
      Serial.println("Servo: ON (Shutters Open)");
    }
    if (!relayState) {
      digitalWrite(relayPin, LOW);
      relayState = true;
      Serial.println("Relay: ON (Fan ON)");
    }
  } else {
    if (servoState) {
      myServo.write(0);
      servoState = false;
      Serial.println("Servo: OFF (Shutters Closed)");
    }
    if (relayState) {
      digitalWrite(relayPin, HIGH);
      relayState = false;
      Serial.println("Relay: OFF (Fan OFF)");
    }
  }
}
