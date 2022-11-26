#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Network credentials
#define WIFI_SSID "H 18"
#define WIFI_PASSWORD "ardalena"

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

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;
const float GYROSCOPE_DANGER_THRESHOLD = 0.5235987756; // 30 degrees
const float GYROSCOPE_WARNING_THRESHOLD = 0.1745329252; // 30 degrees

const float ACCELEROMETER_DANGER_THRESHOLD = 1; // 1 m/s^2
const float ACCELEROMETER_WARNING_THRESHOLD = 0.5; // 0.5 m/s^2

float gyro_base_x = 0, gyro_base_y = 0, gyro_base_z = 0;
float accel_base_x = 0, accel_base_y = 0, accel_base_z = 0;

float gyro_x = 0, gyro_y = 0, gyro_z = 0;
float accel_x = 0, accel_y = 0, accel_z = 0;

#include <Servo.h>
#define SERVO_ACTUATOR 13
Servo door_servo;

#include <Adafruit_Fingerprint.h>
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&Serial2);
uint8_t id;
unsigned int fingerprint_trying_limit = 0;
const unsigned int FINGERPRINT_TRY_LIMIT = 3;

#define PIR_SENSOR_1 12
#define PIR_SENSOR_2 14
#define PIR_SENSOR_3 27
#define PIR_SENSOR_4 26

#define BUZZER 18

boolean PIR_sensor_status_1 = false, PIR_sensor_status_2 = false, PIR_sensor_status_3 = false, PIR_sensor_status_4 = false;
unsigned int PIR_stack = 0; // Untuk menyimpan seberapa lama PIR terdeteksi
const unsigned int PIR_STACK_LIMIT = 10;

const unsigned long fingerprint_event_time = 50;
const unsigned long gyroscope_event_time = 1000;
const unsigned long PIR_event_time = 1000;
const unsigned long firebase_event_time = 5000;
unsigned long fingerprint_t = 0, gyroscope_t = 0, PIR_t = 0, firebase_t = 0;

const byte STATUS_STANDBY = 0;
const byte STATUS_WARNING = 1;
const byte STATUS_DANGER = 2;
const byte STATUS_DOOR_OPENED = 3;

byte system_status = STATUS_STANDBY;

void setup() {
  Serial.begin(115200);
  while (!Serial) // Menunggu serial terkoneksi
    delay(10);

  // Inisialisasi PIR Sensor
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
}

void loop() {
  // PIR_handler();
  fingerprint_handler();
  gyroscope_handler();
  firebase_handler();
  // Serial.println("Is fingerprint handler blocking thread?");
  // delay(1000);
}

void change_status(byte STATUS) {
  system_status = STATUS;
  switch(STATUS) {
    case STATUS_STANDBY:
      Serial.println("STATUS: STANDBY");
      buzzer_turn_off();
      close_door();
      break;
    case STATUS_WARNING:
      Serial.println("STATUS: WARNING");
      buzzer_fire_beep();
      break;
    case STATUS_DANGER:
      Serial.println("STATUS: DANGER");
      buzzer_fire_long();
      break;
    case STATUS_DOOR_OPENED:
      Serial.println("STATUS: DOOR OPENED");
      buzzer_turn_off();
      open_door();
      break;
  }
}

void PIR_handler() {
  if(millis() - PIR_t >= PIR_event_time) {
    PIR_t = millis();
    boolean PIR_status_1 = digitalRead(PIR_SENSOR_1);
    boolean PIR_status_2 = digitalRead(PIR_SENSOR_2);
    boolean PIR_status_3 = digitalRead(PIR_SENSOR_3);
    boolean PIR_status_4 = digitalRead(PIR_SENSOR_4);

    PIR_sensors_troubleshoot();

    if(PIR_status_1 || PIR_status_2 || PIR_status_3 || PIR_status_4) {
      PIR_stack++;
      if(PIR_stack >= PIR_STACK_LIMIT) { // Lebih dari 10 detik
        change_status(STATUS_DANGER);
      } else {
        change_status(STATUS_WARNING);
      }
    } else {
      PIR_stack = 0;
      change_status(STATUS_STANDBY);
    }
  }
}

void fingerprint_handler() {
  if(millis() - fingerprint_t >= fingerprint_event_time) {
    fingerprint_t = millis();
    uint8_t fingerprint_check_result = getFingerprintID();

    if(fingerprint_check_result == FINGERPRINT_NOTFOUND) {
      fingerprint_trying_limit++;

      if(fingerprint_trying_limit >= FINGERPRINT_TRY_LIMIT) {
        change_status(STATUS_DANGER);
      } else {
        change_status(STATUS_WARNING);
      }
    } else if (fingerprint_check_result == FINGERPRINT_OK) {
      Serial.println("Pintu dibukaaaa!!!!");
      if(system_status == STATUS_STANDBY) {
        change_status(STATUS_DOOR_OPENED);
      } else {
        change_status(STATUS_STANDBY);
      }
    }
  }
}

void gyroscope_handler() {
  if(millis() - gyroscope_t >= gyroscope_event_time) {
    gyroscope_t = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accel_x = a.acceleration.x;
    accel_y = a.acceleration.y;
    accel_z = a.acceleration.z;

    gyro_x = g.gyro.x;
    gyro_y = g.gyro.y;
    gyro_z = g.gyro.z;

    if(
      (abs(accel_x - accel_base_x) >= ACCELEROMETER_DANGER_THRESHOLD) || 
      (abs(accel_y - accel_base_y) >= ACCELEROMETER_DANGER_THRESHOLD) || 
      (abs(accel_z - accel_base_z) >= ACCELEROMETER_DANGER_THRESHOLD) || 
      (abs(gyro_x - gyro_base_x) >= GYROSCOPE_DANGER_THRESHOLD) || 
      (abs(gyro_y - gyro_base_y) >= GYROSCOPE_DANGER_THRESHOLD) || 
      (abs(gyro_z - gyro_base_z) >= GYROSCOPE_DANGER_THRESHOLD) 
    ) {
      change_status(STATUS_DANGER);
    }
    else if (
      (abs(accel_x - accel_base_x) >= ACCELEROMETER_WARNING_THRESHOLD) || 
      (abs(accel_y - accel_base_y) >= ACCELEROMETER_WARNING_THRESHOLD) || 
      (abs(accel_z - accel_base_z) >= ACCELEROMETER_WARNING_THRESHOLD) || 
      (abs(gyro_x - gyro_base_x) >= GYROSCOPE_WARNING_THRESHOLD) || 
      (abs(gyro_y - gyro_base_y) >= GYROSCOPE_WARNING_THRESHOLD) || 
      (abs(gyro_z - gyro_base_z) >= GYROSCOPE_WARNING_THRESHOLD)
    ) {
      change_status(STATUS_WARNING);
    }   
  }
}

void firebase_handler() {
  if (Firebase.ready() && signupOK && (millis() - firebase_t > firebase_event_time)){
    firebase_t = millis();

    // Get status first before set to local value
    if (Firebase.RTDB.getInt(&fbdo, "system_status")) {
      if (fbdo.dataType() == "int") {
        int firebase_system_status = fbdo.intData();
        Serial.println(firebase_system_status);
        change_status(firebase_system_status);
      }
    }
    else {
      Serial.println(fbdo.errorReason());
    }

    firebase_set_int("system_status", system_status);
    firebase_set_float("accelerometer/x", accel_x);
    firebase_set_float("accelerometer/y", accel_y);
    firebase_set_float("accelerometer/z", accel_z);
    firebase_set_float("gyroscope/x", gyro_x);
    firebase_set_float("gyroscope/y", gyro_y);
    firebase_set_float("gyroscope/z", gyro_z);
    firebase_set_bool("PIR/0", PIR_sensor_status_1);
    firebase_set_bool("PIR/1", PIR_sensor_status_2);
    firebase_set_bool("PIR/2", PIR_sensor_status_3);
    firebase_set_bool("PIR/3", PIR_sensor_status_4);
  }
}

// Init Firebase
void initialize_firebase() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.waitForConnectResult() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
    // Serial.print("\nWiFi status: ");
    // Serial.println(WiFi.status());
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
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
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
  while(!finger.verifyPassword()) {
    Serial.println("Sensor fingerprint tidak ditemukan :(");
    delay(1000);
  }
  Serial.println("Fingerprint sensor ditemukan!");

  Serial.println(F("Membaca parameter sensor"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor tidak menyimpan data fingerprint, tolong jalankan enroll terlebih dahulu!");
  }
  else {
    Serial.println("Menunggu jari...");
    Serial.print("Sensor menyimpan "); Serial.print(finger.templateCount); Serial.println(" templates");
  }
}


/**
 * Inisialisasi sensor gyroscope
 */
void initialize_gyroscope_sensor() {
  while(!mpu.begin()) {
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

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel_base_x = a.acceleration.x;
  accel_base_y = a.acceleration.y;
  accel_base_z = a.acceleration.z;

  gyro_base_x = g.gyro.x;
  gyro_base_y = g.gyro.y;
  gyro_base_z = g.gyro.z;
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
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  //return finger.fingerID;
  return p;
}

void gyroscope_sensor_reading() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(1000);
}

void PIR_sensors_troubleshoot() {
  PIR_sensor_status_1 = digitalRead(PIR_SENSOR_1);
  PIR_sensor_status_2 = digitalRead(PIR_SENSOR_2);
  PIR_sensor_status_3 = digitalRead(PIR_SENSOR_3);
  PIR_sensor_status_4 = digitalRead(PIR_SENSOR_4);
  Serial.print("PIR1: ");
  Serial.print(PIR_sensor_status_1);
  Serial.print("; PIR2: ");
  Serial.print(PIR_sensor_status_2);
  Serial.print("; PIR3: ");
  Serial.print(PIR_sensor_status_3);
  Serial.print("; PIR4: ");
  Serial.println(PIR_sensor_status_4);
}

bool firebase_set_int(String endpoint, int value) {
  if(Firebase.RTDB.setInt(&fbdo, endpoint, value)) {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    return true;
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    return false;
  }
}

bool firebase_set_bool(String endpoint, bool value) {
  if(Firebase.RTDB.setBool(&fbdo, endpoint, value)) {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    return true;
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    return false;
  }
}

bool firebase_set_float(String endpoint, float value) {
  if(Firebase.RTDB.setFloat(&fbdo, endpoint, value)) {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    return true;
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    return false;
  }
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
  for (int pos = 115; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    door_servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15ms for the servo to reach the position
  }
  door_servo.detach();
}

void open_door() {
  door_servo.attach(SERVO_ACTUATOR);
  for (int pos = 0; pos <= 115; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    door_servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15ms for the servo to reach the position
  }
  door_servo.detach();
}
