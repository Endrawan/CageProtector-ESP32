#include "Fingerprint.h"

Fingerprint::Fingerprint() {
  _status = 0;
  _enroll_status = 1;
}

Fingerprint::Fingerprint(int status, int enroll_status) {
  _status = status;
  _enroll_status = enroll_status;
}

int Fingerprint::getStatus() {
  return _status;
}

int Fingerprint::getEnrollStatus() {
  return _enroll_status;
}

void Fingerprint::setStatus(int status) {
  _status = status;
}

void Fingerprint::setEnrollStatus(int enroll_status) {
  _enroll_status = enroll_status;
}