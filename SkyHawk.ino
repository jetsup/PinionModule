//  1 Channel Receiver
//  PWM output on pin D5

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>

#define MAX_PAYLOAD_SIZE 31

int servoPwmDutyCycle = 0;
unsigned long openTime = 0, prevSendTime = 0;
bool wasOpenDigital = false;

Servo servo;
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

char gpsDataTx[100];

static const uint32_t GPSBaud = 115200;
double BASE_LAT, BASE_LON, BASE_ALT;

struct Signal {
  byte analogServoValueByte;
  bool releaseCarriage;
};

Signal data;

const uint64_t pipeIn = 000322;
RF24 radio(5, 2);

void ResetData() {
  data.analogServoValueByte = 0;  // Define the inicial value of each data input. | Veri girişlerinin başlangıç değerleri
  data.releaseCarriage = false;
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  // Set the pins for each PWM signal | Her bir PWM sinyal için pinler belirleniyor.

  servo.attach(4);

  ResetData();  // Configure the NRF24 module  | NRF24 Modül konfigürasyonu
  if (!radio.begin()) { Serial.println("NRF INIT FAILED"); }
  radio.setPayloadSize(100);
  // radio.openReadingPipe(1, pipeIn);

  //
  radio.openWritingPipe(pipeIn);
  //

  radio.setChannel(100);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  // The lowest data rate value for more stable communication
  radio.setPALevel(RF24_PA_MAX);    // Output power is set for maximum
  radio.startListening();           // Start the radio comunication for receiver

  while (!gps.location.isValid() && !gps.altitude.isValid()) {
    // just loop until it gets the geo coordinates of the drone
    Serial.printf("X: %d %d\n", gps.location.isValid(), gps.altitude.isValid());
    smartDelay(200);
  }
  BASE_LAT = gps.location.lat();
  BASE_LON = gps.location.lng();
  BASE_ALT = gps.altitude.meters();
}

unsigned long lastRecvTime = 0;

void loop() {
  receiveControlCommand();
  unsigned long now = millis();
  // if (now - lastRecvTime > 1000) {
  //   ResetData();  // Signal lost.. Reset data
  // }
  servoPwmDutyCycle = map(data.analogServoValueByte, 0, 255, 1000, 2000);  // pin D5 (PWM signal)

  if (data.releaseCarriage) {
    openTime = millis();
    wasOpenDigital = true;
    servo.writeMicroseconds(2000 /*open*/);
    data.releaseCarriage = false;
  } else {
    if (!wasOpenDigital && millis() - openTime > 1000) {
      servo.writeMicroseconds(servoPwmDutyCycle);  // Write the PWM signal
    }
  }

  if (wasOpenDigital && millis() - openTime >= 1000) {
    servo.writeMicroseconds(1000);
    wasOpenDigital = false;
  }

  if (gps.location.isValid()) {

    double courseToBase =
      TinyGPSPlus::courseTo(
        gps.location.lat(),
        gps.location.lng(),
        BASE_LAT,
        BASE_LON);

    unsigned long distanceToBase =
      (unsigned long)TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        BASE_LAT,
        BASE_LON);

    const char* mTime = parseTime(gps.time);

    // Serial.printf("%lf,%lf,%lf|%lf,%lf,%s,%lf,%lf,%lf,%d\n",
    //               BASE_LAT,
    //               BASE_LON,
    //               BASE_ALT,
    //               gps.location.lat(),
    //               gps.location.lng(),
    //               mTime,
    //               parseAltitude(gps.altitude),
    //               parseSpeed(gps.speed),
    //               courseToBase,
    //               distanceToBase /*metres*/
    // );

    // char* gpsDataTx;
    sprintf(gpsDataTx, "%lf,%lf,%lf|%lf,%lf,%s,%lf,%lf,%lf,%d\n",
            BASE_LAT,
            BASE_LON,
            BASE_ALT,
            gps.location.lat(),
            gps.location.lng(),
            mTime,
            parseAltitude(gps.altitude),
            parseSpeed(gps.speed),
            courseToBase,
            distanceToBase /*metres*/
    );

    delete[] mTime;
    // if (millis() - prevSendTime > 100) {
      sendGpsData();
      // prevSendTime = millis();
      // Serial.printf("%s\n", gpsDataTx);
    // }
  }

  // sendGpsData();
}

void splitAndTransmit(String data) {
  Serial.print("$");
  Serial.println(data);
  radio.openWritingPipe(pipeIn);
  radio.stopListening();
  smartDelay(10);

  // Calculate the number of messages needed to transmit the data
  int numMessages = (data.length() + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;

  // Loop over each message index and transmit the message
  for (int i = 0; i < numMessages; i++) {
    // Determine the start and end indices of the current slice of data
    int startIndex = i * MAX_PAYLOAD_SIZE;
    int endIndex = ((i + 1) * MAX_PAYLOAD_SIZE) < data.length() ? (i + 1) * MAX_PAYLOAD_SIZE : data.length();  //min((i + 1) * MAX_PAYLOAD_SIZE, data.length());

    // Extract the 32-byte message from the data
    String message = data.substring(startIndex, endIndex);

    // Convert the String message to a char array
    char messageArray[MAX_PAYLOAD_SIZE];
    message.toCharArray(messageArray, MAX_PAYLOAD_SIZE);

    // Transmit the message
    radio.write(&messageArray, sizeof(messageArray));

    // Print the transmitted message (optional)
    // Serial.print("#");
    // Serial.print(i + 1);
    // Serial.print("/");
    // Serial.print(numMessages);
    // Serial.print(": ");
    // Serial.println(message);
  }
}

void receiveControlCommand() {
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();
  smartDelay(10);
  while (radio.available()) {
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();  // Receive the data | Data alınıyor
  }
}

void sendGpsData() {
  // const char* gpsData = "51.055,-9.12,123.89,0820";
  // Serial.printf(">> %s", gpsDataTx);

  radio.openWritingPipe(pipeIn);
  radio.stopListening();
  smartDelay(10);

  String dd = String(gpsDataTx);
  dd.replace("\n", "");
  dd = "<<" + dd + ">>";

  // radio.write(gpsData, strlen(gpsData));
  // radio.write(gpsDataTx, sizeof(gpsDataTx));

  // Serial.print("$");
  // Serial.println(dd);
  splitAndTransmit(dd);
}

static double parseSpeed(TinyGPSSpeed& speed) {
  if (!speed.isValid()) {
    return -1.0;
  }
  return speed.mps();
}

static double parseAltitude(TinyGPSAltitude& a) {
  if (!a.isValid()) {
    return -1.0;
  }
  return a.meters();
}

static const char* parseTime(TinyGPSTime& t) {
  if (!t.isValid()) {
    return "HH:MM:ss";
  }
  char* sz = new char[32];  // remember to delete the pointer where this function is being called
  sprintf(sz, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());

  return sz;
}

// This custom version of delay() ensures that the gps object is being "fed".
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}