#include "Wire.h"
#include <MPU6050_light.h>
#include <PID_v1.h>
#include <L298N.h>

MPU6050 mpu(Wire);

double setpoint, input, output;

double kp = 5, ki = 0, kd = 0;

const unsigned int IN1 = 4;
const unsigned int IN2 = 5;
const unsigned int EN1 = 3;
const unsigned int IN3 = 8;
const unsigned int IN4 = 7;
const unsigned int EN2 = 6;

L298N motor1(EN1, IN1, IN2);
L298N motor2(EN2, IN3, IN4);

PID tune(&input, &output, &setpoint, kp, ki, kd, DIRECT);


void setup() {

  Serial.begin(38400);
  Wire.begin();
  setpoint = 0;
  mpu.begin();
  mpu.calcOffsets();

  motor1.setSpeed(0);
  motor2.setSpeed(0);
  tune.SetOutputLimits(-255, 255);
  tune.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.update();

  input = mpu.getAngleX();
  tune.Compute();
  driveMotors(output);
}

void driveMotors(double o) {
  if (o > 0) {
    motor1.backward();
    motor2.backward();
  } else if (o < 0) {
    motor1.forward();
    motor2.forward();
  } else {
    motor1.stop();
    motor2.stop();
  }
  motor1.setSpeed((int)abs(o));
  motor2.setSpeed((int)abs(o));
}