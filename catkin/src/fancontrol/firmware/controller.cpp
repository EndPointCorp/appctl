#include <ros.h>
#include <std_msgs/Float32.h>

#include <Arduino.h>
#include <Servo.h>

#include <math.h>

const int LEFT_PIN = 5;
const int RIGHT_PIN = 6;
const int MIN_DUTY_CYCLE = 1000;
const int MAX_DUTY_CYCLE = 2000;

ros::NodeHandle nh;
Servo leftMotor;
Servo rightMotor;

int transform(const float& n, const int& lower, const int& upper) {
  return floor(lower + (n * (upper - lower)));
}
float clamp(const float& n, const float& lower, const float& upper) {
  return fmin(fmax(n, lower), upper);
}
int signalToDutyCycle(float signal) {
  signal = clamp(signal, 0.0, 1.0);
  return transform(signal, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
}

void leftCallback(const std_msgs::Float32& msg) {
  leftMotor.writeMicroseconds(signalToDutyCycle(msg.data));
}
void rightCallback(const std_msgs::Float32& msg) {
  rightMotor.writeMicroseconds(signalToDutyCycle(msg.data));
}

ros::Subscriber<std_msgs::Float32> leftSub("/fancontrol/left/speed", leftCallback);
ros::Subscriber<std_msgs::Float32> rightSub("/fancontrol/right/speed", rightCallback);

void setup() {
  leftMotor.attach(LEFT_PIN, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
  leftMotor.writeMicroseconds(MIN_DUTY_CYCLE);

  rightMotor.attach(RIGHT_PIN, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
  rightMotor.writeMicroseconds(MIN_DUTY_CYCLE);

  nh.initNode();
  nh.subscribe(leftSub);
  nh.subscribe(rightSub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
