#include "PID_t.h"

const int resistorPin = A5;   // analog input (photoresistor)
const int ledPin = 3;         // PWM output (LED)
const int potPin = A0;        // potentiometer to define setpoint

PID_t pid;

double Kp = 0.8;
double Ki = 0.5;
double Kd = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(resistorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(potPin, INPUT);

  pid.setTunings(Kp, Ki, Kd);
  pid.setSampleTime(10);
  pid.setMode(AUTOMATIC);
  pid.setType(ABSOLUTE);
  pid.setDirection(DIRECT);
  pid.setAntiOvershootStyle(CLASSIC);
  pid.setPrecision(NORMAL);
  pid.setOutputLimits(0, 1023);
}

void loop() {
  double reading = analogRead(resistorPin); // Light sensor
  
  double target = analogRead(potPin); // Potentiometer defines setpoint

  pid.setInput(reading);
  pid.setSetpoint(target);

  analogWrite(ledPin, map(pid.compute(), 0, 1023, 0, 255));
  ///*
  // Debug
  Serial.print("Input: ");
  Serial.print(reading);
  Serial.print(" | Setpoint: ");
  Serial.print(target);
  Serial.print(" | PID Output: ");
  Serial.println(pid.getOutput());
  //*/
}
