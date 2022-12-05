#ifndef Fingerprint_h
#define Fingerprint_h

class Fingerprint {
  private:
    int _status, _enroll_status;
  public:
    Fingerprint();
    Fingerprint(int status, int enroll_status);
    int getStatus();
    int getEnrollStatus();
    void setStatus(int status);
    void setEnrollStatus(int enroll_status);
};

#endif