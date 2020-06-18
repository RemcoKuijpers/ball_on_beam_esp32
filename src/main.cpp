#include <Arduino.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <TrivialKalmanFilter.h>
#include <PID_v1.h>

#define dirPin 4
#define stepPin 2

#define DT_COVARIANCE_RK 2
#define DT_COVARIANCE_QK 0.3

int pos, output;
int stepMode = 4;
double Setpoint, Input, Output;
double Kp=0.5*stepMode, Ki=0, Kd=0;

VL53L0X sensor;
TrivialKalmanFilter<float> filter(DT_COVARIANCE_RK, DT_COVARIANCE_QK);
TaskHandle_t core0;
PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double getDistance(){
  return filter.update(sensor.readRangeSingleMillimeters());
}

void stepClockwise(){
    digitalWrite(dirPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2);
    pos++;
}

void stepCounterClockwise(){
    digitalWrite(dirPin, LOW);
    delayMicroseconds(2);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2);
    pos--;
}

void stepperTest(){
  for (output=0; output<400; output++){
    delayMicroseconds(5000);
  }
  delay(1000);
  for (output=400; output>0; output--){
    delayMicroseconds(4000);
  }
  delay(1000);
}

void motorControl(void * pvParameters){
  for(;;){
    while (output - pos > 0){
      stepClockwise();
      delay(2);
    }

    while (output - pos < 0){
      stepCounterClockwise();
      delay(2);
    }
    vTaskDelay(1);
  }
}

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.startContinuous();

  xTaskCreatePinnedToCore(motorControl, "motor_control", 10000, NULL, 1, &core0, 0);

  Input = getDistance();
  Setpoint = 200;
  PID.SetSampleTime(1);
  PID.SetOutputLimits(-50*stepMode,50*stepMode);
  PID.SetMode(AUTOMATIC);
}

void loop() {
  Input = getDistance();
  if (Input > 500){
    Input = 500;
  }

  PID.Compute();
  output = int(Output);

  Serial.print(Input);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  Serial.println(pos);
}