#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include "Adafruit_MPRLS.h"
#include <SPI.h>
#include  <TFT_eSPI.h> // Loads the library itself
#include <PID_v1.h>

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
#define SOLENOID_PIN  37
#define MOTOR_PIN  39
#define LED_PIN 13
#define UP_BUTTON 42
#define DOWN_BUTTON 41
#define SLEEP_PIN 35
#define BL_PIN 9

Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
TFT_eSPI tft = TFT_eSPI();

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//int venipuncture_pressure = 20; //should be 75
int interruptCounter;
float calibration_value;

// PID Setup
double Setpoint = 25;
double Input, Output;
double Kp = 3, Ki = 0.05, Kd = 1; // Example PID tuning values, adjust as needed
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void IRAM_ATTR onTime() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);
  digitalWrite(SOLENOID_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(SLEEP_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin(1,2);

  // Configure Prescaler to 80, as our timer runs @ 80Mhz
  // Giving an output of 80,000,000 / 80 = 1,000,000 ticks / second
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTime, true);
  // Fire Interrupt every 30m ticks, so 30s
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

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
  
  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PWM range for motor control

  // LCD
  tft.init();
  tft.fillScreen(0x00);
  digitalWrite(SLEEP_PIN, HIGH);
  digitalWrite(BL_PIN, HIGH);
}

void loop() {
  float pressure_mmHg = (mpr.readPressure()-calibration_value)*0.75;
  Serial.println(pressure_mmHg);

  if (pressure_mmHg >= Setpoint+3){
    digitalWrite(SOLENOID_PIN, HIGH);
    // digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(SOLENOID_PIN, LOW);
    // digitalWrite(LED_PIN, LOW);
  }
  bool button_1 = digitalRead(UP_BUTTON);
  bool button_2 = digitalRead(DOWN_BUTTON);
  if (button_1 == HIGH){
    Setpoint = Setpoint - 1;
  }
  else if (button_2 == HIGH){
    Setpoint = Setpoint + 1;
  }
  
  Input = pressure_mmHg;
  myPID.Compute();
  analogWrite(MOTOR_PIN, Output);

  tft.drawString(String(Setpoint), 50, 150, 6);
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    tft.drawString(String(pressure_mmHg), 50, 100, 6);
    Serial.println("Interrupt Triggered");
    }
}
