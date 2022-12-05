#include "MPU6050.h"
#include <Arduino.h>

MPU6050::MPU6050() {
  _accelerometer = Axis(0, 0, 0);
  _gyroscope = Axis(0, 0, 0);
}

MPU6050::MPU6050(Axis accelerometer, Axis gyroscope) {
  _accelerometer = accelerometer;
  _gyroscope = gyroscope;
}

Axis& MPU6050::getAccelerometer() {
  return _accelerometer;
}

Axis& MPU6050::getGyroscope() {
  return _gyroscope;
}

void MPU6050::setAccelerometer(Axis accelerometer) {
  _accelerometer = accelerometer;
}

void MPU6050::setGyroscope(Axis gyroscope) {
  _gyroscope = gyroscope;
}

bool MPU6050::isExceedThreshold(MPU6050 currentMPU6050, float accelThreshold, float gyroThreshold) {
  if(
      (abs(currentMPU6050.getAccelerometer().getX() - _accelerometer.getX()) >= accelThreshold) || 
      (abs(currentMPU6050.getAccelerometer().getY() - _accelerometer.getY()) >= accelThreshold) || 
      (abs(currentMPU6050.getAccelerometer().getZ() - _accelerometer.getZ()) >= accelThreshold) || 
      (abs(currentMPU6050.getGyroscope().getX() - _gyroscope.getX()) >= gyroThreshold) || 
      (abs(currentMPU6050.getGyroscope().getY() - _gyroscope.getY()) >= gyroThreshold) || 
      (abs(currentMPU6050.getGyroscope().getZ() - _gyroscope.getZ()) >= gyroThreshold) 
    ) {
      return true;
    }
    else {
      return false;
    }
}