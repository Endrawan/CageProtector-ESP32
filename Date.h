#ifndef Date_h
#define Date_h
#include <Arduino.h>
using namespace std;

class Date {
  private:
    int _year, _month, _day, _hour, _minute, _second;
  public:
    Date();
    void setDateTimeClientString(String ntpTime);
    void setDateFromFirebase(String firebaseTime);
    void setDate(int year, int month, int day, int hour, int minute, int second);
    String printToFirebaseFormat();
    bool isLatestThan(Date d);
    int getYear();
    int getMonth();
    int getDay();
    int getHour();
    int getMinute();
    int getSecond();
};

#endif