/*!
 * mprls_simpletest.ino
 *
 * A basic test of the sensor with default settings
 * 
 * Designed specifically to work with the MPRLS sensor from Adafruit
 * ----> https://www.adafruit.com/products/3965
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
 
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include "Adafruit_MPRLS.h"
#include <SPI.h>
#include  <TFT_eSPI.h> // Loads the library itself

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
#define SOLENOID_PIN  27
#define MOTOR_PIN  15
#define LED_PIN 13
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
TFT_eSPI tft = TFT_eSPI();

float calibrationValue;

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, HIGH);
  digitalWrite(MOTOR_PIN, LOW);
  Serial.begin(115200);
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");
  calibrationValue = mpr.readPressure();

  // LCD
  tft.init();
  tft.fillScreen(0x00);
}

void loop() {
  float pressure_mmHg = (mpr.readPressure()-calibrationValue)*0.75;
  Serial.print("Pressure (mmHg): "); Serial.println(pressure_mmHg);
  Serial.print("Pressure (PSI): "); Serial.println(pressure_mmHg / 68.947572932);
  if (pressure_mmHg >= 100){
    digitalWrite(MOTOR_PIN, LOW);
    digitalWrite(SOLENOID_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_PIN, HIGH);
    digitalWrite(SOLENOID_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
  }
  tft.drawString(String(pressure_mmHg), 50, 100, 6);
  delay(1000);
}