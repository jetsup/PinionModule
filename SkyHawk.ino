#include <ESP32Servo.h>
#include <SPI.h>

#include "utils.h"

Servo servo;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  // Set the pins for each PWM signal

  servo.attach(4);

  ResetData();  // Configure the NRF24 module
  if (!radio.begin()) {
    Serial.println("NRF INIT FAILED");
  }

  radio.openWritingPipe(pipeIn);

  radio.setChannel(100);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  // The lowest data rate value for more
                                    // stable communication
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

void loop() {
  receiveControlCommand();
  unsigned long now = millis();

  servoPwmDutyCycle = map(data.analogServoValueByte, 0, 255, 1000, 2000);

  if (data.releaseCarriage) {
    openTime = millis();
    wasOpenDigital = true;
    packageReleased = true;
    servo.writeMicroseconds(2000 /*open*/);
    data.releaseCarriage = false;
  } else {
    if (!wasOpenDigital && millis() - openTime > 1000) {
      servo.writeMicroseconds(servoPwmDutyCycle);  // Write the PWM signal

      if (servoPwmDutyCycle > 1500) {
        // past a certain point, the package is released
        packageReleased = true;
      }
    }
  }

  if (wasOpenDigital && millis() - openTime >= AUTO_CLOSE_PINION_DELAY) {
    servo.writeMicroseconds(2000);
    wasOpenDigital = false;
  }

  if (gps.location.isValid()) {
    double courseToBase = TinyGPSPlus::courseTo(
        gps.location.lat(), gps.location.lng(), BASE_LAT, BASE_LON);

    unsigned long distanceToBase = (unsigned long)TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(), BASE_LAT, BASE_LON);

    const char* mTime = parseTime(gps.time);

    sprintf(gpsDataTx, "%lf,%lf,%lf|%lf,%lf,%s,%lf,%lf,%lf,%d\n", BASE_LAT,
            BASE_LON, BASE_ALT, gps.location.lat(), gps.location.lng(), mTime,
            parseAltitude(gps.altitude), parseSpeed(gps.speed), courseToBase,
            distanceToBase /*metres*/
    );
    currentLat = (float)gps.location.lat();
    currentLng = (float)gps.location.lng();
    sprintf(currentTime, "%s", mTime);
    currentAlt = (float)parseAltitude(gps.altitude);
    currentSpeed = (float)parseSpeed(gps.speed);
    currentCourse = (float)courseToBase;
    currentDistanceFromBase = distanceToBase;

    delete[] mTime;

    sendGpsData();
  }
}
