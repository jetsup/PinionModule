#ifndef UTILS_H_
#define UTILS_H_

#include <Arduino.h>
#include <RF24.h>
#include <TinyGPSPlus.h>
#include <nRF24L01.h>

#define MAX_PAYLOAD_SIZE 31
#define AUTO_CLOSE_PINION_DELAY 2000

extern int servoPwmDutyCycle;
extern unsigned long openTime;
extern unsigned long prevSendTime;
extern bool wasOpenDigital;

extern char gpsDataTx[100];

static const uint32_t GPSBaud = 115200;
extern double BASE_LAT, BASE_LON, BASE_ALT;

extern double currentLat, currentLng;
extern float currentAlt, currentCourse, currentSpeed;
extern long currentDistanceFromBase;
extern char currentTime[32];

struct Signal {
  byte analogServoValueByte;
  bool releaseCarriage;
};

extern Signal data;
extern HardwareSerial gpsSerial;
extern TinyGPSPlus gps;

extern RF24 radio;

static const uint64_t pipeIn = 000322;
extern unsigned long lastRecvTime;

void ResetData();
void transmitData(String data);
void receiveControlCommand();
void sendGpsData();
double parseSpeed(TinyGPSSpeed& speed);
double parseAltitude(TinyGPSAltitude& a);
const char* parseTime(TinyGPSTime& t);
void smartDelay(unsigned long ms);
#endif  // UTILS_H_