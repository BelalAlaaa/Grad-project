#include <SimpleKalmanFilter.h>


SimpleKalmanFilter kf1(18, 5, 0.025);  // For sensor 1 
SimpleKalmanFilter kf2(18, 5, 0.025);  // For sensor 2

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  // Read and filter both sensors
  int raw1 = analogRead(A0);
  int filtered1 = kf1.updateEstimate(raw1);
  
  int raw2 = analogRead(A0); // modify the pin numb
  int filtered2 = kf2.updateEstimate(raw2);
  
  // Output format for Serial Plotter:
  // raw1 filtered1 raw2 filtered2
  Serial.print(raw1);
  Serial.print(" ");
  Serial.print(filtered1);
  Serial.print(" ");
  Serial.print(raw2);
  Serial.print(" ");
  Serial.println(filtered2);
  
  delay(100);  // 10 samples/second
}