#include <DHT.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>
#include <SimpleKalmanFilter.h>
#include <ArduinoJson.h>

// === Pin Definitions ===
#define DHTPIN1 27
#define DHTPIN2 5
#define SOIL_PIN 34
#define RELAY_PIN 26
#define DHTTYPE DHT11


// === Pin Definitions ===
#define DHTPIN3 18           // DHT22 sensor connected to GPIO 18     // Define the type of DHT sensor
const int fanPin = 14;      // Fan control pin via PWM on GPIO 14

// === DHT Object ===
DHT dht_bwat(DHTPIN3, DHT22);

// === PWM Configuration ===
const int pwmChannel = 0;
const int pwmFreq = 25000;       // 25 kHz for quiet fan control
const int pwmResolution = 8;     // 8-bit resolution (0-255 PWM)

const char* ssid = "green";
const char* password = "greenhousepwd123";

// === Server Config ===
const char* serverUrl = "http://192.168.8.10:5000/receive";
const char* espUsername = "Ventilation_System_ESP";
const char* espPassword = "password";

#define BUFFER_SIZE 50

bool systemActive = false;

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
  float lastRawTemp = 0;
  float lastRawHum = 0;

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
      kalmanHum(0.5, 1.0, 0.01) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
      tempBuffer[i] = 0;
      humBuffer[i] = 0;
    }
  }

  void update() {
    float rawTemp = dht->readTemperature();
    float rawHum = dht->readHumidity();
    if (isnan(rawTemp) || isnan(rawHum)) return;

    lastRawTemp = rawTemp;
    lastRawHum = rawHum;

    tempBuffer[bufferIndex] = rawTemp;
    humBuffer[bufferIndex] = rawHum;
    lastFilteredTemp = kalmanTemp.updateEstimate(rawTemp);
    lastFilteredHum = kalmanHum.updateEstimate(rawHum);

    Serial.print(name); Serial.print(" Raw Temp: "); Serial.print(rawTemp);
    Serial.print(" | Filtered Temp: "); Serial.println(lastFilteredTemp);
    Serial.print(name); Serial.print(" Raw Hum: "); Serial.print(rawHum);
    Serial.print(" | Filtered Hum: "); Serial.println(lastFilteredHum);

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

  float getFilteredTemperature() { return lastFilteredTemp; }
  float getFilteredHumidity() { return lastFilteredHum; }
  float getLastRawTemperature() { return lastRawTemp; }
  float getLastRawHumidity() { return lastRawHum; }
};

// === Global Variables ===
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
KalmanSensor sensor1(&dht1, "Sensor 1");
KalmanSensor sensor2(&dht2, "Sensor 2");
SimpleKalmanFilter kalmanSoil(0.5, 1.0, 0.01);

float rawSoil = 0;
float filteredSoil = 0;

unsigned long previousSensorMillis = 0;
unsigned long previousNetworkMillis = 0;
const unsigned long sensorInterval = 10000;
const unsigned long networkInterval = 30000;

void setup() {
  Serial.begin(115200);
  dht1.begin();
  dht2.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

    // Initialize DHT sensor
  dht_bwat.begin();

  // Initialize PWM on fanPin
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(fanPin, pwmChannel);
  setupWiFi();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;
    Serial.println("Reading sensors...");
    sensor1.update();
    sensor2.update();
    rawSoil = analogRead(SOIL_PIN);
    filteredSoil = kalmanSoil.updateEstimate((float)rawSoil);
    Serial.print("Raw Soil Moisture: "); Serial.println(rawSoil);
    Serial.print("Filtered Soil Moisture: "); Serial.println(filteredSoil);
    controlFanWithDHT22();
  }

  if (currentMillis - previousNetworkMillis >= networkInterval) {
    previousNetworkMillis = currentMillis;
    Serial.println("Sending data to server...");
    sendSensorData();
    controlDevices();
  }
}

void controlDevices() {
  static bool lastState = false;
  if (systemActive && !lastState) {
    digitalWrite(RELAY_PIN, HIGH);
    lastState = true;
    Serial.println("Irrigation system ACTIVATED.");
  } else if (!systemActive && lastState) {
    digitalWrite(RELAY_PIN, LOW);
    lastState = false;
    Serial.println("Irrigation system DEACTIVATED.");
  }
}

String buildJSONArray(float values[], int size) {
  String json = "[";
  for (int i = 0; i < size; i++) {
    json += String(values[i], 2);
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

void setupWiFi() {

  // For ESP32 #1
  IPAddress local_IP(192, 168, 8, 30);
  IPAddress gateway(192, 168, 8, 1);   
  IPAddress subnet(255, 255, 255, 0);

  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());
}

void sendSensorData() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(serverUrl);
  String authHeader = "Basic " + base64::encode(String(espUsername) + ":" + espPassword);
  http.addHeader("Authorization", authHeader);
  http.addHeader("Content-Type", "application/json");

  const int numSensors = 5;
  int sensorIDs[numSensors] = {3001, 3002, 3005, 3003, 3004};

  float temperature[numSensors] = {
    sensor1.getLastRawTemperature(), 0.0, 0.0, sensor2.getLastRawTemperature(), 0.0
  };
  float filteredTemperature[numSensors] = {
    sensor1.getFilteredTemperature(), 0.0, 0.0, sensor2.getFilteredTemperature(), 0.0
  };
  float humidity[numSensors] = {
    0.0, sensor1.getLastRawHumidity(), 0.0, 0.0, sensor2.getLastRawHumidity()
  };
  float filteredHumidity[numSensors] = {
    0.0, sensor1.getFilteredHumidity(), 0.0, 0.0, sensor2.getFilteredHumidity()
  };
  float soilMoisture[numSensors] = {
    0.0, 0.0, rawSoil, 0.0, 0.0
  };
  float filteredSoilMoisture[numSensors] = {
    0.0, 0.0, filteredSoil, 0.0, 0.0
  };

  String actuatorState = systemActive ? "on" : "off";

  String payload = "{";
  payload += "\"DeviceName\":\"Ventilation_System_ESP\",";
  payload += "\"DeviceID\":3000,";
  payload += "\"ActuatorID\":3009,";
  payload += "\"ActuatorState\":\"" + actuatorState + "\",";
  payload += "\"SensorID\":" + buildJSONArray(sensorIDs, numSensors) + ",";
  payload += "\"temperature\":" + buildJSONArray(temperature, numSensors) + ",";
  payload += "\"filtered_temperature\":" + buildJSONArray(filteredTemperature, numSensors) + ",";
  payload += "\"humidity\":" + buildJSONArray(humidity, numSensors) + ",";
  payload += "\"filtered_humidity\":" + buildJSONArray(filteredHumidity, numSensors) + ",";
  payload += "\"soil_moisture\":" + buildJSONArray(soilMoisture, numSensors) + ",";
  payload += "\"filtered_soil_moisture\":" + buildJSONArray(filteredSoilMoisture, numSensors);
  payload += "}";

  Serial.println("Sending Payload:");
  Serial.println(payload);

  http.setTimeout(10000);
  int responseCode = http.POST(payload);

  if (responseCode > 0) {
    String response = http.getString();
    Serial.println("Server Response: " + response);

    const char* res = response.c_str();
    if (strstr(res, "Irrigation_ON")) {
      Serial.println("Turning ON the irrigation system");
      systemActive = true;
      controlDevices();
    } else if (strstr(res, "Irrigation_OFF")) {
      Serial.println("Turning OFF the irrigation system");
      systemActive = false;
      controlDevices();
    } else {
      Serial.println("No valid irrigation command found in response.");
    }

  } else {
    Serial.print("Failed to send data: ");
    Serial.println(responseCode);
  }

  http.end();
}


void controlFanWithDHT22() {
  float temperatureC = dht_bwat.readTemperature();  // Read temperature in Celsius

  if (isnan(temperatureC)) {
    Serial.println("Failed to read from DHT sensor!");
    delay(1000);
    return;
  }

  int fanSpeed = map(temperatureC, 20, 50, 0, 255);
  fanSpeed = constrain(fanSpeed, 0, 255);
  ledcWrite(pwmChannel, fanSpeed);
  Serial.print("bwat temperature: ");
  Serial.println(temperatureC);
  Serial.println("bwatfan speed: ");
  Serial.println(fanSpeed);
}
