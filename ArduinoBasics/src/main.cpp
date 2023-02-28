#include <Arduino.h>
#include <RGBLed.h>

#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>

const int ledRPin = 19;
const int ledGPin = 18;
const int ledBPin = 17;

const int i2c_sda = 5;
const int i2c_scl = 6;

const long fadeTime = 3000;
const long fadeSteps = 256;

const int powerDACPin = 45;
const int powerSD = 32;


RGBLed led(ledRPin, ledGPin, ledBPin, RGBLed::COMMON_CATHODE);
Adafruit_I2CDevice dev = Adafruit_I2CDevice(0x18, &Wire);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup() {
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Team RevvoX hacking with the Toniebox =)");
  Wire.begin(i2c_sda, i2c_scl);
  
  if (!dev.begin(true)) {
    Serial.println("No I2C device found...");
  }
  if (!lis.begin(0x18) && !lis.begin(0x19)) {
    Serial.println("LIS3DH not found...");
  }

  pinMode(powerDACPin, OUTPUT);
  //pinMode(powerSD, OUTPUT); //Triggers reset

  digitalWrite(powerDACPin, 1);
  //digitalWrite(powerSD, 0);
}

void loop() {

  Serial.print("R");
  led.crossFade(RGBLed::RED, RGBLed::GREEN, fadeSteps, fadeTime);
  Serial.print("G");
  led.crossFade(RGBLed::GREEN, RGBLed::BLUE, fadeSteps, fadeTime);
  Serial.print("B");
  led.crossFade(RGBLed::BLUE, RGBLed::RED, fadeSteps, fadeTime);
  Serial.println("");
}