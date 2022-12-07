#ifndef Fingerprint_h
#define Fingerprint_h

class Fingerprint {
  private:
    int _status, _enroll_status, _steps;
  public:
    Fingerprint();
    Fingerprint(int status, int enroll_status);
    int getSteps();
    int getStatus();
    int getEnrollStatus();
    void setSteps(int steps);
    void setStatus(int status);
    void setEnrollStatus(int enroll_status);
};

#endif