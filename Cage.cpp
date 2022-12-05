#include "Cage.h"
#include <Arduino.h>

Cage::Cage() {
  _base_mpu6050 = MPU6050();
  _mpu6050 = MPU6050();
  _system_status = 0;
  _fingerprint = Fingerprint();
  _last_updated_arduino = "10-04-2020 10:51:20";
  _last_updated_android = "10-04-2020 10:51:20";
  _buzzer_status = false;
  _alert_status = true;
}

bool* Cage::getPIR() {
  return _PIR;
}

MPU6050& Cage::getBaseMpu6050() {
  return _base_mpu6050;
}

MPU6050& Cage::getMpu6050() {
  return _mpu6050;
}

int Cage::getSystemStatus() {
  return _system_status;
}

Fingerprint& Cage::getFingerprint() {
  return _fingerprint;
}

String Cage::getLastUpdatedArduino() {
  return _last_updated_arduino;
}

String Cage::getLastUpdatedAndroid() {
  return _last_updated_android;
}

bool Cage::getBuzzerStatus() {
  return _buzzer_status;
}

bool Cage::getAlertStatus() {
  return _alert_status;
}


bool Cage::getPIR(int index) {
  return _PIR[index];
}

void Cage::setPIR(bool value1, bool value2, bool value3, bool value4) {
  _PIR[0] = value1;
  _PIR[1] = value2;
  _PIR[2] = value3;
  _PIR[3] = value4;
}

void Cage::setBaseMpu6050(MPU6050 base_mpu6050) {
  _base_mpu6050 = base_mpu6050;
}

void Cage::setMpu6050(MPU6050 mpu6050) {
  _mpu6050 = mpu6050;
}

void Cage::setSystemStatus(int system_status) {
  _system_status = system_status;
}

void Cage::setFingerprint(Fingerprint fingerprint) {
  _fingerprint = fingerprint;
}

void Cage::setLastUpdatedArduino(String last_updated_arduino) {
  _last_updated_arduino = last_updated_arduino;
}

void Cage::setLastUpdatedAndroid(String last_updated_android) {
  _last_updated_android = last_updated_android;
}

void Cage::setBuzzerStatus(bool buzzer_status) {
  _buzzer_status = buzzer_status;
}

void Cage::setAlertStatus(bool alert_status) {
  _alert_status = alert_status;
}