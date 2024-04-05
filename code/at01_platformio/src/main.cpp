#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include "Adafruit_MPRLS.h"
#include <SPI.h>
#include  <TFT_eSPI.h> // Loads the library itself
#include <PID_v1.h>

// setup enum with all menu page options
enum pageType { homePage,
                venepuncture,
                occlusion,
                bicepOcclusion,
                thighOcclusion };
enum pageType currentPage = homePage;  // default is home
#define homePageCount 2

// Function prototypes
void homePageDetails(void);                                              // function for menu
void venepunctureDetails(void);                                          // function for menu
void bicepOcclusionDetails(void);                                        // function for menu
void thighOcclusionDetails(void);                                        // function for menu
void applyPressure(int maxPressure, int maxTime, char modeDetails[60]);  // apply pressure function
bool buttonPressed(int button);                                          // helper function for menu
bool buttonNotPressed(int button);                                       // helper function for menu

// flags
bool applyPressureActive = false;  // flag for pressure
bool updateDisplay = true;         // flag for updating display

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

// Pressure and Time Variables
const float VENEPUNCTURE_PRESSURE = 60;
const float BICEP_PRESSURE = 203;
const float THIGH_PRESSURE = 300;
float startPressure = 0;
float currentPressure = 0;
float targetPressure = 0;

const unsigned long VENEPUNCTURE_TIME = 60000;
const unsigned long BICEP_TIME = 5400000;
const unsigned long THIGH_TIME = 7200000;
unsigned long startTime = 0;
unsigned long currentTime = 0;
unsigned long targetTime = 0;

// modes
char mode[60];

Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
TFT_eSPI tft = TFT_eSPI();

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//int venipuncture_pressure = 20; //should be 75
int interruptCounter;
float calibration_value;

// PID Setup
double maxPressure = 25;
double Input, Output;
// Input = currentPressure;
double Kp = 3, Ki = 0.05, Kd = 1; // Example PID tuning values, adjust as needed
PID myPID(&Input, &Output, &maxPressure, Kp, Ki, Kd, DIRECT);

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
  startTime = millis();  //initial start time
  // Serial.println(currentPage);
  switch (currentPage) {
    case homePage: homePageDetails(); break;
    case venepuncture: venepunctureDetails(); break;
    case occlusion: occlusionDetails(); break;
    case bicepOcclusion: bicepOcclusionDetails(); break;
    case thighOcclusion: thighOcclusionDetails(); break;
  }

  if (applyPressureActive) {
    applyPressure(targetPressure, targetTime, mode);
  }

  // float pressure_mmHg = (mpr.readPressure()-calibration_value)*0.75;
  // Serial.println(pressure_mmHg);

  // if (pressure_mmHg >= Setpoint+3){
  //   digitalWrite(SOLENOID_PIN, HIGH);
  //   // digitalWrite(LED_PIN, HIGH);
  // }
  // else {
  //   digitalWrite(SOLENOID_PIN, LOW);
  //   // digitalWrite(LED_PIN, LOW);
  // }
  // bool button_1 = digitalRead(UP_BUTTON);
  // bool button_2 = digitalRead(DOWN_BUTTON);
  // if (button_1 == HIGH){
  //   Setpoint = Setpoint - 1;
  // }
  // else if (button_2 == HIGH){
  //   Setpoint = Setpoint + 1;
  // }
  
  // Input = pressure_mmHg;
  // myPID.Compute();
  // analogWrite(MOTOR_PIN, Output);

  // tft.drawString(String(Setpoint), 50, 150, 6);
  // if (interruptCounter > 0) {
  //   portENTER_CRITICAL(&timerMux);
  //   interruptCounter--;
  //   portEXIT_CRITICAL(&timerMux);
  //   tft.drawString(String(pressure_mmHg), 50, 100, 6);
  //   Serial.println("Interrupt Triggered");
    // }
}

// ========================================================================
// ||                            HOME PAGE                               ||
// ========================================================================
void homePageDetails(void) {

  // flag for updating display
  updateDisplay = true;

  // track when entering beginning of loop
  uint32_t loopStartMS;

  // track button states
  bool UP_BUTTON_WAS_PRESSED = false;
  bool DOWN_BUTTON_WAS_PRESSED = false;

  // inner loop
  while (true) {

    // capture start time
    loopStartMS = millis();

    if (updateDisplay) {

      // clear update flag
      updateDisplay = false;

      // clear display
      tft.fillScreen(TFT_BLACK);

      // create main menu display
      tft.drawString("[ MAIN MENU ]", 10, 20, 4);
      tft.drawString("{ -------------------------------- }", 10, 50, 2);
      tft.drawString("Option 1: Venepuncture [UP]", 10, 70, 2);
      tft.drawString("Option 2: Limb Occlusion [DOWN]", 10, 90, 2);
      tft.drawString("{ -------------------------------- }", 10, 110, 2);
    }

    // capture the button down states
    if (buttonPressed(UP_BUTTON)) {
      UP_BUTTON_WAS_PRESSED = true;  // set up button flag
    }
    if (buttonPressed(DOWN_BUTTON)) {
      DOWN_BUTTON_WAS_PRESSED = true;  // set down button flag
    }

    // move pointer down
    if (DOWN_BUTTON_WAS_PRESSED && buttonNotPressed(DOWN_BUTTON)) {
      currentPage = occlusion;
      Serial.println("Current Selection: OCCLUSION");
      updateDisplay = true;             // set display flag
      DOWN_BUTTON_WAS_PRESSED = false;  // clear down button flag
      return;
    }

    // move pointer up
    if (UP_BUTTON_WAS_PRESSED && buttonNotPressed(UP_BUTTON)) {
      currentPage = venepuncture;
      Serial.println("Current Selection: VENEPUNCTURE");
      updateDisplay = true;           // set display flag
      UP_BUTTON_WAS_PRESSED = false;  // clear up button flag
      return;
    }
  }
}


// ========================================================================
// ||                            TOOLS - BUTTON PRESSED                  ||
// ========================================================================
bool buttonPressed(int button) {
  return digitalRead(button) == HIGH && digitalRead(button) == HIGH;
}


// ========================================================================
// ||                        TOOLS - BUTTON NOT PRESSED                  ||
// ========================================================================
bool buttonNotPressed(int button) {
  return digitalRead(button) == LOW && digitalRead(button) == LOW;
}


// ========================================================================
// ||                            VENEPUNCTURE PAGE                       ||
// ========================================================================
void venepunctureDetails(void) {
  while (true) {
    // clear display
    tft.fillScreen(TFT_BLACK);

    // page display
    tft.drawString("[VENEPUNCTURE SELECTED]", 10, 20, 2);
    tft.drawString("{ -------------------------------- }", 10, 50, 2);
    tft.drawString("Target pressure = 60 mmHg,", 10, 70, 2);
    tft.drawString("applied for 60 seconds.", 10, 90, 2);
    tft.drawString("{ -------------------------------- }", 10, 110, 2);
    delay(5000);  // show target details for 5 seconds

    // setting pressure, time and mode
    targetPressure = VENEPUNCTURE_PRESSURE;
    targetTime = VENEPUNCTURE_TIME;
    // Copy the string to mode
    strcpy(mode, "VENEPUNCTURE, ARM");

    // start device
    applyPressureActive = true;

    applyPressure(targetPressure, targetTime, mode);
  }
}

// ========================================================================
// ||                             OCCLUSION PAGE                         ||
// ========================================================================
void occlusionDetails(void) {
  // flag for updating display
  updateDisplay = true;

  // track when entering beginning of loop
  uint32_t loopStartMS;

  // track button states
  bool UP_BUTTON_WAS_PRESSED = false;
  bool DOWN_BUTTON_WAS_PRESSED = false;

  // inner loop
  while (true) {

    // capture start time
    loopStartMS = millis();

    if (updateDisplay) {

      // clear update flag
      updateDisplay = false;

      // clear display
      tft.fillScreen(TFT_BLACK);

      // page display
      tft.drawString("[OCCLUSION SELECTED]", 10, 20, 2);
      tft.drawString("{ -------------------------------- }", 10, 50, 2);
      tft.drawString("Option 1: Bicep [UP]", 10, 70, 2);
      tft.drawString("Option 2: Thigh [DOWN]", 10, 90, 2);
      tft.drawString("{ -------------------------------- }", 10, 110, 2);
    }

    // capture the button down states
    if (buttonPressed(UP_BUTTON)) {
      UP_BUTTON_WAS_PRESSED = true;  // set up button flag
    }
    if (buttonPressed(DOWN_BUTTON)) {
      DOWN_BUTTON_WAS_PRESSED = true;  // set down button flag
    }

    // move pointer down
    if (DOWN_BUTTON_WAS_PRESSED && buttonNotPressed(DOWN_BUTTON)) {
      currentPage = bicepOcclusion;
      Serial.println("Current Selection: BICEP OCCLUSION");
      updateDisplay = true;             // set display flag
      DOWN_BUTTON_WAS_PRESSED = false;  // clear down button flag
      return;
    }

    // move pointer up
    if (UP_BUTTON_WAS_PRESSED && buttonNotPressed(UP_BUTTON)) {
      currentPage = thighOcclusion;
      Serial.println("Current Selection: THIGH OCCLUSION");
      updateDisplay = true;           // set display flag
      UP_BUTTON_WAS_PRESSED = false;  // clear up button flag
      return;
    }
  }
}

// ========================================================================
// ||                            BICEP OCCLUSION PAGE                    ||
// ========================================================================
void bicepOcclusionDetails(void) {
  while (true) {
    // clear display
    tft.fillScreen(TFT_BLACK);

    // page display
    tft.drawString("[BICEP OCCLUSION SELECTED]", 10, 20, 2);
    tft.drawString("{ -------------------------------- }", 10, 50, 2);
    tft.drawString("Target pressure = 203 mmHg,", 10, 70, 2);
    tft.drawString("applied for 1.5 hours.", 10, 90, 2);
    tft.drawString("{ -------------------------------- }", 10, 110, 2);
    delay(5000);  // show target details for 5 seconds

    // setting pressure and time
    targetPressure = VENEPUNCTURE_PRESSURE;
    targetTime = VENEPUNCTURE_TIME;
    // Copy the string to mode
    strcpy(mode, "OCCLUSION, BICEP");

    // start device
    applyPressureActive = true;

    // Continue the loop to update pressure display
    applyPressure(targetPressure, targetTime, mode);
  }
}

// ========================================================================
// ||                            THIGH OCCLUSION PAGE                    ||
// ========================================================================
void thighOcclusionDetails(void) {
  while (true) {
    // clear display
    tft.fillScreen(TFT_BLACK);

    // page display
    tft.drawString("[THIGH OCCLUSION SELECTED]", 10, 20, 2);
    tft.drawString("{ -------------------------------- }", 10, 50, 2);
    tft.drawString("Target pressure = 300 mmHg,", 10, 70, 2);
    tft.drawString("applied for 2 hours.", 10, 90, 2);
    tft.drawString("{ -------------------------------- }", 10, 110, 2);
    delay(5000);  // show target details for 5 seconds

    // setting pressure and time
    targetPressure = VENEPUNCTURE_PRESSURE;
    targetTime = VENEPUNCTURE_TIME;
    // Copy the string to mode
    strcpy(mode, "OCCLUSION, THIGH");

    // start device
    applyPressureActive = true;

    // Continue the loop to update pressure display
    applyPressure(targetPressure, targetTime, mode);
  }
}

// ========================================================================
// ||                        APPLY PRESSURE                              ||
// ========================================================================
void applyPressure(int maxPressure, int maxTime, char* modeDetails) {
  // Concatenate the modeDetails string with "Mode: "
  char modeMessage[60];  // Declare a character array to hold the concatenated message
  strcpy(modeMessage, "Mode: ");
  strcat(modeMessage, modeDetails);
  updateDisplay = true;
  while (true) {
    if (updateDisplay) {
      // clear display
      tft.fillScreen(TFT_BLACK);
      // display mode as title
      tft.drawString("{ -------------------------------- }", 10, 10, 2);
      tft.drawString(modeMessage, 10, 30, 2);
      // display pressure information
      tft.drawString("{ -------------------------------- }", 10, 50, 2);
      tft.drawString("Current Pressure [mmHg]:", 10, 70, 2);
      tft.drawString("{ -------------------------------- }", 10, 110, 2);
      tft.drawString("Target Pressure [mmHg]:", 10, 130, 2);
      tft.drawString(String(maxPressure), 10, 150, 2);
      tft.drawString("{ -------------------------------- }", 10, 170, 2);
      tft.drawString("Current Time [sec]:", 10, 190, 2);
      tft.drawString("{ -------------------------------- }", 10, 230, 2);
      tft.drawString("Target Time [sec]:", 10, 250, 2);
      tft.drawString(String(maxTime / 1000), 10, 270, 2);
      tft.drawString("{ -------------------------------- }", 10, 290, 2);

      updateDisplay = false;  // Reset update flag
    }

    bool buttonUp = digitalRead(UP_BUTTON);
    bool buttonDown = digitalRead(DOWN_BUTTON);
    if (buttonUp == HIGH) {
      maxPressure = maxPressure + 1;
      updateDisplay = true;
    } else if (buttonDown == HIGH) {
      maxPressure = maxPressure - 1;
      updateDisplay = true;
    }

    Input = currentPressure;
    myPID.Compute();
    analogWrite(MOTOR_PIN, Output);

    if (interruptCounter > 0) {
      portENTER_CRITICAL(&timerMux);
      interruptCounter--;
      portEXIT_CRITICAL(&timerMux);
      float currentTime = millis();
      currentPressure = (mpr.readPressure() - calibration_value) * 0.75;
      char pressureString[10];                         // Assuming the maximum length of the string won't exceed 10 characters
      dtostrf(currentPressure, 6, 2, pressureString);  // Convert to string with 6 characters including 2 decimal points
      tft.drawString(pressureString, 10, 90, 2);       // Display the pressure string
      tft.drawString(String(currentTime / 1000 - 8, 2), 10, 210, 2);
      Serial.println("Interrupt Triggered");
    }

    if (currentPressure >= maxPressure + 3) {
      digitalWrite(SOLENOID_PIN, HIGH);
      // digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(SOLENOID_PIN, LOW);
      // digitalWrite(LED_PIN, LOW);
    }
  }
}
