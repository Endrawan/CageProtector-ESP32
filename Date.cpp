#include "Date.h"
#include <Arduino.h>

Date::Date() {
  _year = 2022;
  _month = 10;
  _day = 10;
  _hour = 10;
  _minute = 10;
  _second = 10;
}


void Date::setDateTimeClientString(String ntpTime) {
  int splitT = ntpTime.indexOf("T");
  String dayStamp = ntpTime.substring(0, splitT);
  String timeStamp = ntpTime.substring(splitT+1, ntpTime.length()-1);

  _year = dayStamp.substring(0, dayStamp.indexOf("-")).toInt();
  _month = dayStamp.substring(dayStamp.indexOf("-") + 1, dayStamp.lastIndexOf("-")).toInt();
  _day = dayStamp.substring(dayStamp.lastIndexOf("-") + 1, dayStamp.length()).toInt();
  _hour = timeStamp.substring(0, timeStamp.indexOf(":")).toInt();
  _minute = timeStamp.substring(timeStamp.indexOf(":") + 1, timeStamp.lastIndexOf(":")).toInt();
  _second = timeStamp.substring(timeStamp.lastIndexOf(":") + 1, timeStamp.length()).toInt();
}

void Date::setDateFromFirebase(String ntpTime) {
  int splitT = ntpTime.indexOf(" ");
  String dayStamp = ntpTime.substring(0, splitT);
  String timeStamp = ntpTime.substring(splitT+1, ntpTime.length());

  _day = dayStamp.substring(0, dayStamp.indexOf("-")).toInt();
  _month = dayStamp.substring(dayStamp.indexOf("-") + 1, dayStamp.lastIndexOf("-")).toInt();
  _year = dayStamp.substring(dayStamp.lastIndexOf("-") + 1, dayStamp.length()).toInt();
  _hour = timeStamp.substring(0, timeStamp.indexOf(":")).toInt();
  _minute = timeStamp.substring(timeStamp.indexOf(":") + 1, timeStamp.lastIndexOf(":")).toInt();
  _second = timeStamp.substring(timeStamp.lastIndexOf(":") + 1, timeStamp.length()).toInt();
}

void Date::setDate(int year, int month, int day, int hour, int minute, int second) {
  _year = year;
  _month = month;
  _day = day;
  _hour = hour;
  _minute = minute;
  _second = second;
}

bool Date::isLatestThan(Date d) {
  if(_year > d.getYear()) {
    return true;
  } else if(_year < d.getYear()) {
    return false;
  }

  if(_month > d.getMonth()) {
    return true;
  } else if(_month < d.getMonth()) {
    return false;
  }

  if(_day > d.getDay()) {
    return true;
  } else if(_day < d.getDay()) {
    return false;
  }

  if(_hour > d.getHour()) {
    return true;
  } else if(_hour < d.getHour()) {
    return false;
  }

  if(_minute > d.getMinute()) {
    return true;
  } else if(_minute < d.getMinute()) {
    return false;
  }

  if(_second > d.getSecond()) {
    return true;
  } else if(_second < d.getSecond()) {
    return false;
  }
  return true;
}

String Date::printToFirebaseFormat() {
  String d = "";
  if((_day / 10) == 0) {
    d += "0";
  }
  d += String(_day);
  d += "-";
  if((_month / 10) == 0) {
    d += "0";
  }
  d+= String(_month);
  d+= "-";
  d+= String(_year);
  d+= " ";
  if((_hour / 10) == 0) {
    d += "0";
  }
  d+= String(_hour);
  d+= ":";
  if((_minute / 10) == 0) {
    d += "0";
  }
  d+= String(_minute);
  d+= ":";
  if((_second / 10) == 0) {
    d += "0";
  }
  d+= String(_second);

  return d;
}

int Date::getYear() {
  return _year;
}

int Date::getMonth() {
  return _month;
}

int Date::getDay() {
  return _day;
}

int Date::getHour() {
  return _hour;
}

int Date::getMinute() {
  return _minute;
}

int Date::getSecond() {
  return _second;
}