#include <Arduino.h>
#include <RGBLed.h>

#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>

#include "FS.h"
#include "SD_MMC.h"

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
const int powerSD = 48;


RGBLed led(ledRPin, ledGPin, ledBPin, RGBLed::COMMON_CATHODE);
Adafruit_I2CDevice dev = Adafruit_I2CDevice(0x19, &Wire);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup() {
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Team RevvoX hacking with the Toniebox =)");
  
  pinMode(wakeupPin, INPUT);
  pinMode(earLeftPin, INPUT);
  pinMode(earRightPin, INPUT);

  pinMode(earRightPin, INPUT);
  pinMode(powerDACPin, OUTPUT); 
  pinMode(resetDACPin, OUTPUT); 

  digitalWrite(resetDACPin, 0);
  digitalWrite(powerDACPin, 1);
  delay(10); 
  digitalWrite(resetDACPin, 1); //DAC IC2 0x18?
  Wire.setPins(i2c_sda, i2c_scl);
  if (!lis.begin(0x19)) {
    Serial.println("LIS3DH not found...");
  }

  pinMode(powerSD, OUTPUT); //Triggers reset
  digitalWrite(powerSD, 0);
  delay(100);

  //37	SPICLK_P	Power	SD	Low = Power on
  //38	GPIO33	DAT2	SD	
  //39	GPIO34	DAT3	SD	
  //40	GPIO35	CLK	SD	
  //41	GPIO36	DAT0	SD	
  //42	GPIO37	DAT1	SD	
  //--	--	--	--	--
  //43	GPIO38	CMD	SD

  if (!SD_MMC.setPins(35, 38, 36, 37, 33, 34)) {
    Serial.println("Couldn't set SD-Pins...");
  } else if(!SD_MMC.begin()) {
    Serial.println("Card mount failed");
  }
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
}