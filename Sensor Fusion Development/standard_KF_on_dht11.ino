#include <DHT.h>
#include <SimpleKalmanFilter.h>

#define DHTPIN D2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// Initial Kalman filter parameters
SimpleKalmanFilter kalmanTemp(4, 2.0, 0.1);  // q=1.0, e=1.0, dt=0.1
SimpleKalmanFilter kalmanHum(25, 1.0, 0.1);

const long SERIAL_REFRESH_TIME = 2000;
long refresh_time;

const int BUFFER_SIZE = 10;
float tempBuffer[BUFFER_SIZE];
float humBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// Track current noise values since library doesn't provide getters
float currentTempQ = 1.0;
float currentHumQ = 1.0;
float currentTempE = 1.0;
float currentHumE = 1.0;

unsigned long startTime;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  dht.begin();
  startTime = millis();
  
  // Initialize buffers
  for (int i = 0; i < BUFFER_SIZE; i++) {
    tempBuffer[i] = 0;
    humBuffer[i] = 0;
  }
  
  Serial.println("RawTemp,FilteredTemp,RawHum,FilteredHum");
}

void loop() {
  if (millis() > refresh_time) {
    float rawTemp = dht.readTemperature();
    float rawHum = dht.readHumidity();

    if (isnan(rawTemp) || isnan(rawHum)) {
      Serial.println("Sensor error! Check wiring.");
      refresh_time = millis() + SERIAL_REFRESH_TIME;
      return;
    }

    // Add to circular buffers
    tempBuffer[bufferIndex] = rawTemp;
    humBuffer[bufferIndex] = rawHum;
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    
    if (bufferIndex == 0) {
      bufferFilled = true;
    }
    
    // Get filtered values
    float filteredTemp = kalmanTemp.updateEstimate(rawTemp);
    float filteredHum = kalmanHum.updateEstimate(rawHum);
    
    // Adaptive parameter tuning
    if (bufferFilled) {
      float tempMean = calcMean(tempBuffer, BUFFER_SIZE);
      float tempStd = calcStdDev(tempBuffer, BUFFER_SIZE, tempMean);
      float humMean = calcMean(humBuffer, BUFFER_SIZE);
      float humStd = calcStdDev(humBuffer, BUFFER_SIZE, humMean);
      
      // Constrain standard deviations
      tempStd = constrain(tempStd, 0.1f, 2.0f);
      humStd = constrain(humStd, 0.2f, 4.0f);
      
      // Check gaps
      float tempGap = abs(rawTemp - filteredTemp);
      float humGap = abs(rawHum - filteredHum);
      
      // Update parameters with smoothing
      if (tempGap > 2.0f) {
        float newQ = tempStd * 0.8f;
        float newE = tempStd;
        currentTempQ = 0.7f * currentTempQ + 0.3f * newQ;
        currentTempE = 0.7f * currentTempE + 0.3f * newE;
        
        // Recreate filter with new parameters
        kalmanTemp = SimpleKalmanFilter(currentTempQ, currentTempE, 0.1);
      }
      
      if (humGap > 5.0f) {
        float newQ = humStd * 0.8f;
        float newE = humStd;
        currentHumQ = 0.7f * currentHumQ + 0.3f * newQ;
        currentHumE = 0.7f * currentHumE + 0.3f * newE;
        
        // Recreate filter with new parameters
        kalmanHum = SimpleKalmanFilter(currentHumQ, currentHumE, 0.1);
      }
    }

    // Output results
    Serial.print(rawTemp); Serial.print(",");
    Serial.print(filteredTemp); Serial.print(",");
    Serial.print(rawHum); Serial.print(",");
    Serial.println(filteredHum);
    
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