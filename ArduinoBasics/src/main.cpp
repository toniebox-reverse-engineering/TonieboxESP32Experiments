#include <Arduino.h>
#include <RGBLed.h>

#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>

#include <TRF7962A.h>

#include "FS.h"
#include "SD_MMC.h"

TRF7962A rfid = TRF7962A();

const int wakeupPin = 7;
const int AdcBuchPin = 8;
const int AdcBattPin = 9;

const int earLeftPin = 20;
const int earRightPin = 21;

const int ledRPin = 19;
const int ledGPin = 18;
const int ledBPin = 17;

const int i2c_sda = 5;
const int i2c_scl = 6;

const long fadeTime = 500;
const long fadeSteps = 64;

const int powerDACPin = 45;
const int resetDACPin = 26;
const int powerSD = 47;

const int SD_CMD = 38;
const int SD_CLK = 35;
const int SD_DAT0 = 36;
const int SD_DAT1 = 37;
const int SD_DAT2 = 33;
const int SD_DAT3 = 34;

RGBLed led(ledRPin, ledGPin, ledBPin, RGBLed::COMMON_CATHODE);
Adafruit_I2CDevice dev = Adafruit_I2CDevice(0x19, &Wire);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file) {
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.path(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
     file = root.openNextFile();
  }
}

void setup() {
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  Serial.setDebugOutput(true);
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Team RevvoX hacking with the Toniebox =)");

  Serial.println("Init pins");
  
  pinMode(wakeupPin, INPUT);
  pinMode(earLeftPin, INPUT);
  pinMode(earRightPin, INPUT);

  pinMode(earRightPin, INPUT);
  pinMode(powerDACPin, OUTPUT); 
  pinMode(resetDACPin, OUTPUT); 

  pinMode(powerSD, OUTPUT);
  digitalWrite(powerSD, 0);

  digitalWrite(powerDACPin, 1);
  delay(1);
  digitalWrite(resetDACPin, 0);
  delay(10); 
  digitalWrite(resetDACPin, 1); //DAC IC2 0x18?
  Wire.setPins(i2c_sda, i2c_scl);
  if (!lis.begin(0x19)) {
    Serial.println("LIS3DH not found...");
  }
  Serial.println("LIS3DH ok");

  rfid.begin(1, 13);

  delay(100);
  if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_DAT0, SD_DAT1, SD_DAT2, SD_DAT3)) {
    Serial.println("Couldn't set SD-Pins...");
  } 

  while (!SD_MMC.begin()) {
    Serial.println("Card mount failed");
    delay(2000);
  }

  Serial.println("SD ok");
  delay(500);
  Serial.printf("SD size=%lld, type=%i", SD_MMC.cardSize(), SD_MMC.cardType());
  Serial.println("");
  listDir(SD_MMC, "/", 1);
}

void loop() {
  uint16_t buch = analogRead(AdcBuchPin);
  uint16_t batt = analogRead(AdcBattPin);
  bool wake = digitalRead(wakeupPin);
  bool earLeft = digitalRead(earLeftPin);
  bool earRight = digitalRead(earRightPin);

  
  Serial.print("R");
  led.crossFade(RGBLed::RED, RGBLed::GREEN, fadeSteps, fadeTime);
  Serial.print("G");
  led.crossFade(RGBLed::GREEN, RGBLed::BLUE, fadeSteps, fadeTime);
  Serial.print("B");
  led.crossFade(RGBLed::BLUE, RGBLed::RED, fadeSteps, fadeTime);
  Serial.println("");

  lis.read();
  Serial.printf("LIS3DH: x=%i, y=%i, z=%i", lis.x, lis.y, lis.z);
  Serial.println("...");

  Serial.printf("Buch=%i, Batt=%i, Wake=%i, earL=%i, earR=%i", buch, batt, wake, earLeft, earRight);
  Serial.println();

  TRF7962A::TAG_EVENT tagEvent = rfid.loop();
  if (tagEvent == TRF7962A::TAG_EVENT::TAG_PLACED) {
    uint8_t txtUid[24];
    rfid.getUID(txtUid);
    Serial.printf("Tag detected, UID: %s", txtUid);
    Serial.println();
    led.flash(RGBLed::WHITE, fadeTime);
  } else if (tagEvent == TRF7962A::TAG_EVENT::TAG_REMOVED) {
    Serial.println("Tag removed");
    led.flash(RGBLed::YELLOW, fadeTime);
  } else {
    Serial.printf("Tag result: %i/%i", rfid.getLastResult(), rfid.getLastTrfStatus());
    Serial.println();
  }
}