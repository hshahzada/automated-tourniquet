 
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include "Adafruit_MPRLS.h"
#include <SPI.h>
#include  <TFT_eSPI.h> // Loads the library itself

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
#define SOLENOID_PIN  40
#define MOTOR_PIN  37
#define LED_PIN 13
#define UP_BUTTON 42
#define DOWN_BUTTON 41
#define SLEEP_PIN 35
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
TFT_eSPI tft = TFT_eSPI();

int venipuncture_pressure = 75;
float calibration_value;

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);
  digitalWrite(SOLENOID_PIN, HIGH);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(SLEEP_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin(1,2);
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");
  calibration_value = mpr.readPressure();

  // LCD
  tft.init();
  tft.fillScreen(0x00);
  digitalWrite(SLEEP_PIN, HIGH);
}

void loop() {
  float pressure_mmHg = (mpr.readPressure()-calibration_value)*0.75;
  Serial.println(pressure_mmHg);
  Serial.print("Pressure (PSI): "); Serial.println(pressure_mmHg / 68.947572932);
  if (pressure_mmHg >= venipuncture_pressure){
    digitalWrite(MOTOR_PIN, LOW);
    digitalWrite(SOLENOID_PIN, HIGH);
    // digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(MOTOR_PIN, HIGH);
    digitalWrite(SOLENOID_PIN, HIGH);
    // digitalWrite(LED_PIN, LOW);
  }
  bool button_1 = digitalRead(UP_BUTTON);
  bool button_2 = digitalRead(DOWN_BUTTON);
  if (button_1 == HIGH){
    venipuncture_pressure = venipuncture_pressure - 1;
  }
  else if (button_2 == HIGH){
    venipuncture_pressure = venipuncture_pressure + 1;
  }
  tft.drawString(String(pressure_mmHg), 50, 100, 6);
  tft.drawString(String(venipuncture_pressure), 50, 150, 6);
  // delay(1000);
}
