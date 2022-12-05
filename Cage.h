#ifndef Cage_h
#define Cage_h

#include "MPU6050.h"
#include "Fingerprint.h"
#include <Arduino.h>

class Cage {
  private:
    bool _PIR[4] = {false, false, false, false};
    MPU6050 _base_mpu6050, _mpu6050;
    int _system_status;
    Fingerprint _fingerprint;
    String _last_updated_arduino;
    String _last_updated_android;
    bool _buzzer_status;
    bool _alert_status;

  public:
    Cage();
    bool* getPIR();
    bool getPIR(int index);
    MPU6050& getBaseMpu6050();
    MPU6050& getMpu6050();
    int getSystemStatus();
    Fingerprint& getFingerprint();
    String getLastUpdatedArduino();
    String getLastUpdatedAndroid();
    bool getBuzzerStatus();
    bool getAlertStatus();
    
    void setPIR(bool value1, bool value2, bool value3, bool value4);
    void setBaseMpu6050(MPU6050 base_mpu6050);
    void setMpu6050(MPU6050 mpu6050);
    void setSystemStatus(int system_status);
    void setFingerprint(Fingerprint fingerprint);
    void setLastUpdatedArduino(String last_updated_arduino);
    void setLastUpdatedAndroid(String last_updated_android);
    void setBuzzerStatus(bool buzzer_status);
    void setAlertStatus(bool alert_status);
};

#endif