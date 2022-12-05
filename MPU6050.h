#ifndef MPU6050_h
#define MPU6050_h

#include "Axis.h"

class MPU6050 {
  private:
    Axis _accelerometer, _gyroscope;
  public:
    MPU6050();
    MPU6050(Axis accelerometer, Axis gyroscope);
    Axis& getAccelerometer();
    Axis& getGyroscope();
    void setAccelerometer(Axis accelerometer);
    void setGyroscope(Axis gyroscope);
    bool isExceedThreshold(MPU6050 currentMPU6050, float accelThreshold, float gyroThreshold);
};

#endif