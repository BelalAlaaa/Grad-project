#include <DHT.h>
#include <SimpleKalmanFilter.h>

#define DHTTYPE DHT11
#define BUFFER_SIZE 50
#define SERIAL_REFRESH_TIME 2000

// DHT sensor pin definitions
#define DHTPIN1 D2
#define DHTPIN2 D3
#define DHTPIN3 D4

// DHT sensor objects
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
DHT dht3(DHTPIN3, DHTTYPE);

// Kalman filters for temperature and humidity (3 sensors)
SimpleKalmanFilter kalmanTemp[3] = {
  SimpleKalmanFilter(0.5, 1.0, 0.02),
  SimpleKalmanFilter(0.5, 1.0, 0.02),
  SimpleKalmanFilter(0.5, 1.0, 0.02)
};

SimpleKalmanFilter kalmanHum[3] = {
  SimpleKalmanFilter(0.5, 1.0, 0.01),
  SimpleKalmanFilter(0.5, 1.0, 0.01),
  SimpleKalmanFilter(0.5, 1.0, 0.01)
};

// Adaptive tracking values
float currentTempQ[3] = {0.5, 0.5, 0.5};
float currentHumQ[3] = {0.5, 0.5, 0.5};
float currentTempE[3] = {1.0, 1.0, 1.0};
float currentHumE[3] = {1.0, 1.0, 1.0};

// Circular buffers for noise analysis
float tempBuffer[3][BUFFER_SIZE];
float humBuffer[3][BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

unsigned long refresh_time = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  dht1.begin();
  dht2.begin();
  dht3.begin();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < BUFFER_SIZE; j++) {
      tempBuffer[i][j] = 0;
      humBuffer[i][j] = 0;
    }
  }

  Serial.println("T1,T2,T3,FiltT1,FiltT2,FiltT3,H1,H2,H3,FiltH1,FiltH2,FiltH3");
}

void loop() {
  if (millis() > refresh_time) {
    DHT* sensors[3] = {&dht1, &dht2, &dht3};
    float rawTemp[3], rawHum[3], filtTemp[3], filtHum[3];

    for (int i = 0; i < 3; i++) {
      rawTemp[i] = sensors[i]->readTemperature();
      rawHum[i] = sensors[i]->readHumidity();

      if (isnan(rawTemp[i]) || isnan(rawHum[i])) {
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.println(" error! Check wiring.");
        return;
      }

      // Save to buffer
      tempBuffer[i][bufferIndex] = rawTemp[i];
      humBuffer[i][bufferIndex] = rawHum[i];

      // Apply Kalman filter
      filtTemp[i] = kalmanTemp[i].updateEstimate(rawTemp[i]);
      filtHum[i] = kalmanHum[i].updateEstimate(rawHum[i]);

      if (bufferFilled) {
        float tMean = calcMean(tempBuffer[i], BUFFER_SIZE);
        float tStd = calcStdDev(tempBuffer[i], BUFFER_SIZE, tMean);
        float hMean = calcMean(humBuffer[i], BUFFER_SIZE);
        float hStd = calcStdDev(humBuffer[i], BUFFER_SIZE, hMean);

        tStd = constrain(tStd, 0.1f, 2.0f);
        hStd = constrain(hStd, 0.2f, 4.0f);

        float tGap = abs(rawTemp[i] - filtTemp[i]);
        float hGap = abs(rawHum[i] - filtHum[i]);

        if (tGap > 2.0f) {
          float newQ = tStd * 0.8f;
          float newE = tStd;
          currentTempQ[i] = 0.7f * currentTempQ[i] + 0.3f * newQ;
          currentTempE[i] = 0.7f * currentTempE[i] + 0.3f * newE;
          kalmanTemp[i] = SimpleKalmanFilter(currentTempQ[i], currentTempE[i], 0.1);
        }

        if (hGap > 5.0f) {
          float newQ = hStd * 0.8f;
          float newE = hStd;
          currentHumQ[i] = 0.7f * currentHumQ[i] + 0.3f * newQ;
          currentHumE[i] = 0.7f * currentHumE[i] + 0.3f * newE;
          kalmanHum[i] = SimpleKalmanFilter(currentHumQ[i], currentHumE[i], 0.1);
        }
      }
    }

    // Print data: raw temps, filtered temps, raw hums, filtered hums
    for (int i = 0; i < 3; i++) {
      Serial.print(rawTemp[i]); Serial.print(",");
    }
    for (int i = 0; i < 3; i++) {
      Serial.print(filtTemp[i]); Serial.print(",");
    }
    for (int i = 0; i < 3; i++) {
      Serial.print(rawHum[i]); Serial.print(",");
    }
    for (int i = 0; i < 3; i++) {
      Serial.print(filtHum[i]);
      if (i < 2) Serial.print(",");
    }
    Serial.println();

    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferIndex == 0) bufferFilled = true;

    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }
}

float calcMean(float* data, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return sum / size;
}

float calcStdDev(float* data, int size, float mean) {
  float sumSquaredDiff = 0;
  for (int i = 0; i < size; i++) {
    float diff = data[i] - mean;
    sumSquaredDiff += diff * diff;
  }
  return sqrt(sumSquaredDiff / size);
}

