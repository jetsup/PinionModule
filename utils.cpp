#include "utils.h"

int servoPwmDutyCycle = 0;

unsigned long openTime = 0, prevSendTime = 0;
bool wasOpenDigital = false;

char gpsDataTx[100];

double currentLat, currentLng;
float currentAlt, currentCourse, currentSpeed;
long currentDistanceFromBase;
char currentTime[32];

double BASE_LAT, BASE_LON, BASE_ALT;

unsigned long lastRecvTime = 0;

Signal data;
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
RF24 radio(5, 2);

void ResetData() {
  data.analogServoValueByte =
      0;  // Define the initial value of each data input.
  data.releaseCarriage = false;
}
void transmitData(String data) {
  // Serial.println(data);
  radio.openWritingPipe(pipeIn);
  radio.stopListening();
  smartDelay(10);

  // Calculate the number of messages needed to transmit the data
  int numMessages = (data.length() + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;

  char buffer[32];
  sprintf(buffer, "<B%lf,%lfB>", BASE_LAT, BASE_LON);
  radio.write(buffer, sizeof(buffer));

  sprintf(buffer, "<BA%fBA>", BASE_ALT);
  radio.write(buffer, sizeof(buffer));

  sprintf(buffer, "<C%lf,%lfC>", currentLat, currentLng);
  radio.write(buffer, sizeof(buffer));

  sprintf(buffer, "<CA%fCA>", currentAlt);
  radio.write(buffer, sizeof(buffer));

  sprintf(buffer, "<T%sT>", currentTime);
  radio.write(buffer, sizeof(buffer));

  sprintf(buffer, "<S%fS>", currentSpeed);
  radio.write(buffer, sizeof(buffer));

  sprintf(buffer, "<Q%fQ>", currentCourse);
  radio.write(buffer, sizeof(buffer));

  sprintf(buffer, "<D%dD>", currentDistanceFromBase);
  radio.write(buffer, sizeof(buffer));
}

void receiveControlCommand() {
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();
  smartDelay(10);
  if (radio.available()) {
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();  // Receive the data
    Serial.print("-------------------");
    Serial.print(data.releaseCarriage);
    Serial.print("::");
    Serial.print(data.analogServoValueByte);
    Serial.println("-------------------");
  }
}

void sendGpsData() {
  radio.openWritingPipe(pipeIn);
  radio.stopListening();
  smartDelay(10);

  String dd = String(gpsDataTx);
  dd.replace("\n", "");
  dd = "<$" + dd + "#>";

  transmitData(dd);
}

double parseSpeed(TinyGPSSpeed& speed) {
  if (!speed.isValid()) {
    return -1.0;
  }
  return speed.mps();
}

double parseAltitude(TinyGPSAltitude& a) {
  if (!a.isValid()) {
    return -1.0;
  }
  return a.meters();
}

const char* parseTime(TinyGPSTime& t) {
  if (!t.isValid()) {
    return "HH:MM:ss";
  }
  char* sz = new char[32];  // remember to delete the pointer where this
                            // function is being called
  sprintf(sz, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());

  return sz;
}

// This custom version of delay() ensures that the gps object is being "fed".
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}