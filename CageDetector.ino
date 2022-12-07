#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <NTPClient.h>
#include <Adafruit_Fingerprint.h>
#include <WiFiUdp.h>

#include "Date.h"
#include "Axis.h"
#include "MPU6050.h"
#include "Fingerprint.h"
#include "Cage.h"

// Network credentials
#define WIFI_SSID "ACER 5509"
#define WIFI_PASSWORD "lalalala"

// Insert Firebase project API Key
#define API_KEY "AIzaSyCvdL17w64g-opNXXrChW1J5rHQluw2EQM"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://cageprotector-default-rtdb.asia-southeast1.firebasedatabase.app/"

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

int count = 0;
bool signupOK = false;

Adafruit_MPU6050 mpu;
const float GYROSCOPE_DANGER_THRESHOLD = 0.5235987756;   // 30 degrees
const float GYROSCOPE_WARNING_THRESHOLD = 0.1745329252;  // 10 degrees

const float ACCELEROMETER_DANGER_THRESHOLD = 1;     // 1 m/s^2
const float ACCELEROMETER_WARNING_THRESHOLD = 0.5;  // 0.5 m/s^2

#define SERVO_ACTUATOR 13
Servo door_servo;

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&Serial2);
uint8_t id;
unsigned int fingerprint_trying_limit = 0;
const unsigned int FINGERPRINT_TRY_LIMIT = 3;

#define PIR_SENSOR_1 12
#define PIR_SENSOR_2 14
#define PIR_SENSOR_3 27
#define PIR_SENSOR_4 26

#define BUZZER 18

unsigned int PIR_stack = 0;  // Untuk menyimpan seberapa lama PIR terdeteksi
const unsigned int PIR_STACK_LIMIT = 10;

const unsigned long fingerprint_event_time = 50;
const unsigned long gyroscope_event_time = 1000;
const unsigned long calibrate_mpu6050_event_time = 1000;
const unsigned long PIR_event_time = 1000;
const unsigned long firebase_event_time = 3000;
const unsigned long buzzer_beep_event_time = 1000;
unsigned long fingerprint_t = 0, gyroscope_t = 0, PIR_t = 0, firebase_t = 0, buzzer_t = 0, calibrate_t = 0;

const byte STATUS_STANDBY = 0;
const byte STATUS_WARNING = 1;
const byte STATUS_DANGER = 2;
const byte STATUS_DOOR_OPENED = 3;
const byte STATUS_FINGERPRINT_ENROLL = 5;
const byte STATUS_CALIBRATE_MPU6050 = 6;

const byte FINGERPRINT_STATUS_LISTENING = 0;
const byte FINGERPRINT_STATUS_ENROLL = 1;
const byte FINGERPRINT_ENROLL_STATUS_ENROLLING = 1;
const byte FINGERPRINT_ENROLL_STATUS_FAILED = 3;
const byte FINGERPRINT_ENROLL_STATUS_SUCCESS = 2;

const char* ntpServer = "0.id.pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 0;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

Date prevArduinoTime = Date();
Date arduinoTime = Date();
Date androidTime = Date();

Cage cage = Cage();

void setup() {
  Serial.begin(115200);
  // while (!Serial)  // Menunggu serial terkoneksi
  //   delay(10);

  // // Inisialisasi PIR Sensor
  pinMode(PIR_SENSOR_1, INPUT);
  pinMode(PIR_SENSOR_2, INPUT);
  pinMode(PIR_SENSOR_3, INPUT);
  pinMode(PIR_SENSOR_4, INPUT);

  // Inisialisasi Buzzer
  pinMode(BUZZER, OUTPUT);

  // Inisialisasi Gyroscope
  initialize_gyroscope_sensor();

  // Inisialisasi Fingerprint
  initialize_fingerprint_sensor();

  // Inisialisasi Firebase
  initialize_firebase();

  // Inisialisasi Timestamp
  initialize_timestamp();
}

void loop() {
  // cage.setSystemStatus(STATUS_FINGERPRINT_ENROLL);
  Serial.print("System Status:");
  Serial.println(cage.getSystemStatus());
  switch (cage.getSystemStatus()) {
    case STATUS_STANDBY:
      fingerprint_trying_limit = 0;
      PIR_stack = 0;
      while(cage.getSystemStatus() == STATUS_STANDBY) {
        PIR_handler();
        fingerprint_handler();
        gyroscope_handler();
        buzzer_handler();
        sync_firebase_handler();
      }
      break;
    case STATUS_WARNING:
      while(cage.getSystemStatus() == STATUS_WARNING) {
        PIR_handler();
        fingerprint_handler();
        gyroscope_handler();
        buzzer_handler();
        sync_firebase_handler();
      }
      break;
    case STATUS_DANGER:
      while(cage.getSystemStatus() == STATUS_DANGER) {
        buzzer_handler();
        sync_firebase_handler();
      }
      break;
    case STATUS_DOOR_OPENED:
      open_door();
      cage.setAlertStatus(false);
      while(cage.getSystemStatus() == STATUS_DOOR_OPENED) {
        fingerprint_handler();
        sync_firebase_handler();
      }
      close_door();
      break;
    case STATUS_FINGERPRINT_ENROLL:
      while(cage.getSystemStatus() == STATUS_FINGERPRINT_ENROLL) {
        fingerprint_enroll_handler();
        sync_firebase_handler();
      }
      break;
    case STATUS_CALIBRATE_MPU6050:
      while(cage.getSystemStatus() == STATUS_CALIBRATE_MPU6050) {
        calibrate_mpu6050_handler();
        sync_firebase_handler();
      }
      break;
  }
}

void sync_firebase_handler() {
  if (Firebase.ready() && signupOK && (millis() - firebase_t > firebase_event_time)) {
    firebase_t = millis();
    String last_updated_android;
    if (!firebase_get_string("last_updated_android", last_updated_android)) {
      Serial.println("Tidak bisa mengambil data last_updated_android dari firebase!");
      return;
    }

    cage.setLastUpdatedAndroid(last_updated_android);
    androidTime.setDateFromFirebase(last_updated_android);

    if (androidTime.isLatestThan(prevArduinoTime)) {
      int latestSystemStatus;
      if (!firebase_get_int("system_status", latestSystemStatus)) {
        Serial.println("Tidak bisa mengambil data system_status dari firebase!");
      } else {
        Serial.print("Changed status to ");
        Serial.print(latestSystemStatus);
        Serial.println(" from syncFirebase");
        cage.setSystemStatus(latestSystemStatus);
      }

      bool latestAlertStatus;
      if (!firebase_get_bool("alert_status", latestAlertStatus)) {
        Serial.println("Tidak bisa mengambil data system_status dari firebase!");
      } else {
        cage.setAlertStatus(latestAlertStatus);
      }

      updateMPU6050Value(cage.getBaseMpu6050());
    }

    updateCageDataToFirebase();
  }
}

void updateCageDataToFirebase() {
  String t = timeClient.getFormattedDate();
  arduinoTime.setDateTimeClientString(t);
  cage.setLastUpdatedArduino(arduinoTime.printToFirebaseFormat());

  firebase_set_int("system_status", cage.getSystemStatus());
  firebase_set_float("mpu6050/accelerometer/x", cage.getMpu6050().getAccelerometer().getX());
  firebase_set_float("mpu6050/accelerometer/y", cage.getMpu6050().getAccelerometer().getY());
  firebase_set_float("mpu6050/accelerometer/z", cage.getMpu6050().getAccelerometer().getZ());
  firebase_set_float("mpu6050/gyroscope/x", cage.getMpu6050().getGyroscope().getX());
  firebase_set_float("mpu6050/gyroscope/y", cage.getMpu6050().getGyroscope().getY());
  firebase_set_float("mpu6050/gyroscope/z", cage.getMpu6050().getGyroscope().getZ());
  firebase_set_bool("PIR/0", cage.getPIR(0));
  firebase_set_bool("PIR/1", cage.getPIR(1));
  firebase_set_bool("PIR/2", cage.getPIR(2));
  firebase_set_bool("PIR/3", cage.getPIR(3));
  firebase_set_bool("alert_status", cage.getAlertStatus());
  firebase_set_bool("buzzer_status", cage.getBuzzerStatus());
  firebase_set_string("last_updated_arduino", cage.getLastUpdatedArduino());

  prevArduinoTime.setDateFromFirebase(arduinoTime.printToFirebaseFormat());
}

void PIR_handler() {
  if (millis() - PIR_t >= PIR_event_time) {
    PIR_t = millis();

    bool pir1 = digitalRead(PIR_SENSOR_1);
    bool pir2 = digitalRead(PIR_SENSOR_2);
    bool pir3 = digitalRead(PIR_SENSOR_3);
    bool pir4 = digitalRead(PIR_SENSOR_4);
 
    // Serial.print("PIR1: "); Serial.print(pir1); Serial.print("; PIR2: "); Serial.print(pir2); Serial.print("; PIR3: "); Serial.print(pir3); Serial.print("; PIR4: "); Serial.println(pir3);
    cage.setPIR(pir1, pir2, pir3, pir4);

    if ((pir1 || pir2 || pir3 || pir4) && cage.getAlertStatus()) {
      PIR_stack++;
      if (PIR_stack >= PIR_STACK_LIMIT) {  // Lebih dari 10 detik
        Serial.println("Changed status to danger from PIR Handler");
        cage.setSystemStatus(STATUS_DANGER);
      } else {
        Serial.println("Changed status to warning from PIR Handler");
        cage.setSystemStatus(STATUS_WARNING);
      }
    } else {
      PIR_stack = 0;
      Serial.println("Changed status to standby from PIR Handler");
      cage.setSystemStatus(STATUS_STANDBY);
    }
  }
}

void buzzer_handler() {
  if ((millis() - buzzer_t >= buzzer_beep_event_time) && cage.getSystemStatus() == STATUS_WARNING) {
    buzzer_t = millis();
    cage.setBuzzerStatus(!cage.getBuzzerStatus());
    if(cage.getBuzzerStatus()) { // If buzzer active
      digitalWrite(BUZZER, HIGH);
    } else {
      digitalWrite(BUZZER, LOW);
    }
  } else if (cage.getSystemStatus() == STATUS_DANGER) {
    cage.setBuzzerStatus(true);
    digitalWrite(BUZZER, HIGH);
  } else {
    cage.setBuzzerStatus(false);
    digitalWrite(BUZZER, LOW);
  }
}

void fingerprint_handler() {
  if (millis() - fingerprint_t >= fingerprint_event_time) {
    fingerprint_t = millis();
    uint8_t fingerprint_check_result = getFingerprintID();

    if (fingerprint_check_result == FINGERPRINT_NOTFOUND && cage.getAlertStatus()) {
      fingerprint_trying_limit++;

      if (fingerprint_trying_limit >= FINGERPRINT_TRY_LIMIT) {
        Serial.println("Changed status to danger from fingerprint handler");
        cage.setSystemStatus(STATUS_DANGER);
      } else {
        Serial.println("Changed status to warning from fingerprint handler");
        cage.setSystemStatus(STATUS_WARNING);
      }
    } else if (fingerprint_check_result == FINGERPRINT_OK) {
      fingerprint_trying_limit = 0;
      Serial.println("Pintu dibukaaaa!!!!");
      if (cage.getSystemStatus() != STATUS_DOOR_OPENED) {
        Serial.println("Changed status to door opened from fingerprint handler");
        cage.setSystemStatus(STATUS_DOOR_OPENED);
      } else {
        Serial.println("Changed status to standby from fingerprint handler");
        cage.setSystemStatus(STATUS_STANDBY);
      }
    }
  }
}

void fingerprint_enroll_handler() {
  // firebase_set_int("fingerprint/enroll_status", 0);
  int firebase_enroll_status = 0;
  firebase_get_int("fingerprint/enroll_status", firebase_enroll_status);
  if(firebase_enroll_status == FINGERPRINT_ENROLL_STATUS_ENROLLING) {
    uint8_t fingerprint_enroll_result = fingerprint_enroll();
    if(fingerprint_enroll_result == true) {
      firebase_set_int("fingerprint/enroll_status", FINGERPRINT_ENROLL_STATUS_SUCCESS);
    } else {
      firebase_set_int("fingerprint/enroll_status", FINGERPRINT_ENROLL_STATUS_FAILED);
    }
  }
}

uint8_t fingerprint_enroll() {
  int p = -1;
  uint8_t id = 69;
  Serial.print("Waiting for valid finger to enroll as #"); Serial.println(id); // 1
  firebase_set_int("fingerprint/steps", 1);
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  Serial.println("Remove finger"); // 2
  firebase_set_int("fingerprint/steps", 2);
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  Serial.print("ID "); Serial.println(id);
  p = -1;
  Serial.println("Place same finger again"); // 3
  firebase_set_int("fingerprint/steps", 3);
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  Serial.print("Creating model for #");  Serial.println(id);

  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.println("Prints matched!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    Serial.println("Fingerprints did not match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  Serial.print("ID "); Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.println("Stored!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("Could not store in that location");
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("Error writing to flash");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  firebase_set_int("fingerprint/steps", 4);
  return true;
}

void gyroscope_handler() {
  if (millis() - gyroscope_t >= gyroscope_event_time) {
    gyroscope_t = millis();

    updateMPU6050Value(cage.getMpu6050());

    if (cage.getBaseMpu6050().isExceedThreshold(cage.getMpu6050(), ACCELEROMETER_DANGER_THRESHOLD, GYROSCOPE_DANGER_THRESHOLD) && cage.getAlertStatus()) {
      Serial.println("Changed status to danger from gyroscope handler");
      cage.setSystemStatus(STATUS_DANGER);
    } else {
      if (cage.getBaseMpu6050().isExceedThreshold(cage.getMpu6050(), ACCELEROMETER_WARNING_THRESHOLD, GYROSCOPE_WARNING_THRESHOLD) && cage.getAlertStatus()) {
        Serial.println("Changed status to warning from gyroscope handler");
        cage.setSystemStatus(STATUS_WARNING);
      } else {
        // Serial.println("Changed status to standby from gyroscope handler");
        // cage.setSystemStatus(STATUS_STANDBY);
      }
    }
  }
}

void calibrate_mpu6050_handler() {
  if (millis() - calibrate_t >= calibrate_mpu6050_event_time) {
    calibrate_t = millis();

    updateMPU6050Value(cage.getBaseMpu6050());
    firebase_set_float("base_mpu6050/accelerometer/x", cage.getBaseMpu6050().getAccelerometer().getX());
    firebase_set_float("base_mpu6050/accelerometer/y", cage.getBaseMpu6050().getAccelerometer().getY());
    firebase_set_float("base_mpu6050/accelerometer/z", cage.getBaseMpu6050().getAccelerometer().getZ());
    firebase_set_float("base_mpu6050/gyroscope/x", cage.getBaseMpu6050().getGyroscope().getX());
    firebase_set_float("base_mpu6050/gyroscope/y", cage.getBaseMpu6050().getGyroscope().getY());
    firebase_set_float("base_mpu6050/gyroscope/z", cage.getBaseMpu6050().getGyroscope().getZ());
  }
}

void initialize_timestamp() {
  timeClient.begin();
  timeClient.setTimeOffset(25200);

  while (!timeClient.update()) {
    timeClient.forceUpdate();
    Serial.println("Cannot update timeClient!");
  }

  prevArduinoTime.setDateTimeClientString(timeClient.getFormattedDate());
}

// Init Firebase
void initialize_firebase() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

/**
 * Inisialisasi sensor fingerprint
 */
void initialize_fingerprint_sensor() {
  Serial.println("\n\nAdafruit fingerprint sensor initialization");

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  while (!finger.verifyPassword()) {
    Serial.println("Sensor fingerprint tidak ditemukan :(");
    delay(1000);
  }
  Serial.println("Fingerprint sensor ditemukan!");

  Serial.println(F("Membaca parameter sensor"));
  finger.getParameters();
  Serial.print(F("Status: 0x"));
  Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x"));
  Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: "));
  Serial.println(finger.capacity);
  Serial.print(F("Security level: "));
  Serial.println(finger.security_level);
  Serial.print(F("Device address: "));
  Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: "));
  Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: "));
  Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor tidak menyimpan data fingerprint, tolong jalankan enroll terlebih dahulu!");
  } else {
    Serial.println("Menunggu jari...");
    Serial.print("Sensor menyimpan ");
    Serial.print(finger.templateCount);
    Serial.println(" templates");
  }
}


/**
 * Inisialisasi sensor gyroscope
 */
void initialize_gyroscope_sensor() {
  while (!mpu.begin()) {
    Serial.println("Gagal mencari MPU6050 chip");
    delay(1000);
  }
  Serial.println("MPU6050 ditemukan!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println("Accelerometer range set to: +-8G");
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("Gyro range set to: +- 500 deg/s");
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("Filter bandwidth set to: 21 Hz");
  Serial.println("");
  delay(100);

  updateMPU6050Value(cage.getBaseMpu6050());
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      // Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #");
  Serial.print(finger.fingerID);
  Serial.print(" with confidence of ");
  Serial.println(finger.confidence);

  //return finger.fingerID;
  return p;
}


bool firebase_set_int(String endpoint, int value) {
  if (Firebase.RTDB.setInt(&fbdo, endpoint, value)) {
    // Serial.println("PASSED");
    // Serial.println("PATH: " + fbdo.dataPath());
    // Serial.println("TYPE: " + fbdo.dataType());
    return true;
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    return false;
  }
}

bool firebase_set_bool(String endpoint, bool value) {
  if (Firebase.RTDB.setBool(&fbdo, endpoint, value)) {
    // Serial.println("PASSED");
    // Serial.println("PATH: " + fbdo.dataPath());
    // Serial.println("TYPE: " + fbdo.dataType());
    return true;
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    return false;
  }
}

bool firebase_set_float(String endpoint, float value) {
  if (Firebase.RTDB.setFloat(&fbdo, endpoint, value)) {
    // Serial.println("PASSED");
    // Serial.println("PATH: " + fbdo.dataPath());
    // Serial.println("TYPE: " + fbdo.dataType());
    return true;
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    return false;
  }
}

bool firebase_set_string(String endpoint, String value) {
  if (Firebase.RTDB.setString(&fbdo, endpoint, value)) {
    // Serial.println("PASSED");
    // Serial.println("PATH: " + fbdo.dataPath());
    // Serial.println("TYPE: " + fbdo.dataType());
    return true;
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    return false;
  }
}

bool firebase_get_int(String endpoint, int& returnValue) {
  if (Firebase.RTDB.getInt(&fbdo, endpoint)) {
    if (fbdo.dataType() == "int") {
      returnValue = fbdo.intData();
      // Serial.println(returnValue);
      return true;
    }
  } else {
    Serial.println(fbdo.errorReason());
  }
  return false;
}

bool firebase_get_string(String endpoint, String& returnValue) {
  if (Firebase.RTDB.getString(&fbdo, endpoint)) {
    if (fbdo.dataType() == "string") {
      returnValue = fbdo.stringData();
      // Serial.println(returnValue);
      return true;
    }
  } else {
    Serial.println(fbdo.errorReason());
  }
  return false;
}

bool firebase_get_bool(String endpoint, bool& returnValue) {
  if (Firebase.RTDB.getBool(&fbdo, endpoint)) {
    if (fbdo.dataType() == "boolean") {
      returnValue = fbdo.boolData();
      Serial.println(returnValue);
      return true;
    }
  } else {
    Serial.println(fbdo.errorReason());
  }
  return false;
}

void updateMPU6050Value(MPU6050& mpu6050) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  mpu6050.getAccelerometer().setXYZ(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  mpu6050.getGyroscope().setXYZ(g.gyro.x, g.gyro.y, g.gyro.z);
}

void buzzer_fire_beep() {
  digitalWrite(BUZZER, HIGH);
  delay(1000);
  digitalWrite(BUZZER, LOW);
}

void buzzer_fire_long() {
  digitalWrite(BUZZER, HIGH);
}

void buzzer_turn_off() {
  digitalWrite(BUZZER, LOW);
}

void close_door() {
  door_servo.attach(SERVO_ACTUATOR);
  for (int pos = 115; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees
    door_servo.write(pos);                   // tell servo to go to position in variable 'pos'
    delay(5);                                // waits 15ms for the servo to reach the position
  }
  door_servo.detach();
}

void open_door() {
  door_servo.attach(SERVO_ACTUATOR);
  for (int pos = 0; pos <= 115; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    door_servo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(5);               // waits 15ms for the servo to reach the position
  }
  door_servo.detach();
}