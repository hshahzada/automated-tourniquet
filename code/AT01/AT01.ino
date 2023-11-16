/*
Adafruit Arduino - Lesson 13. DC Motor
*/

#include "HX710B.h"
#include <stdio.h>
#include "xc.h"
#include <p24FJ128GA010.h> // for microcontroller
#include <assert.h>

HX710B pressure_sensor;

int motorPin = 5;
// 1. Declaration of macros and global variables
#define BUTTON _RB8 // change pin number for button as per need 
#define BUTTON_PRESSED() (BUTTON == 0)    // button pressed state
#define BUTTON_RELEASED() (BUTTON == 1)   // button released state
#define LED _RE5 // LED
int TargetPressure, maxTime; 


// 2. Function prototypes
void msDelay(int msec); // declare msDelay function
void UsageMode; // declare UsageMode function
void TargetLimb; // declare TargetLimb function

int main() {
    // 3. Initialization
    AD1PCFG = 0xFFFF;            // Port B pins are set to digital
    CNPU1 = 0x0000;              // No internal (weak) pull-up
    // TRISB binary: 0000 0000 0000 0001, (RB0-RB5 as input, high); hex: 0 0 0 1; 
    // TRISB 
    TRISB = 0x0001; 
    // TRISE binary: 0000 0000 0000 0000, (RE0-RE2 as output, or all as low)
    // TRISE hex: 0 0 0 0
    TRISE = 0; 

    msDelay (50); // wait a few seconds for user to press
    // stage 1: determine usage mode 
    UsageMode(); // call usage mode function
    if (UsageMode() = "OCCLUSION"){
        TargetLimb(); // // call target limb function
    }
    else if (UsageMode() = "VENIPUNCTURE"){
        TargetPressure = 60; // [mmHg] 
        maxTime = 60; // [s], or 1/60 [h]
    }
    else{ // no usage mode determined, try again
        UsageMode(); // call usage mode function
    }
    
    // stage 2: determine target limb 
    if (TargetLimb() = "BICEP"){
        ***
    }
    else if (TargetLimb() = "THIGH"){
        ***
    }
    else{ // no usage mode determined, try again
        TargetLimb(); // call target limb function
    }
    
    return 0;
}

void msDelay (int msec){ // Function 1: Delay Function 
    int m,n;
    for(m=0;m<msec;m++){
        for(n=0;n<50;n++){
        }
    }
}

void UsageMode { // Function 2: checking button for mode of usage
    printf("Please select Mode of Usage: press button for 'Occlusion', or leave unpressed for 'Venipuncture'");
    msDelay(50); // wait for user to select mode
    char Mode[]; 
    while(1){
        if (BUTTON_PRESSED()){
            printf("You have selected: 'Occlusion', please press again to confirm."); 
            msDelay(50); // wait for user to confirm
            if(BUTTON_PRESSED()){
                printf("'Occlusion' Mode is confirmed."); 
                Mode = "OCCLUSION";
                break;
            }
            else{
                printf("'Occlusion' Mode NOT confirmed, please try again."); 
                Mode = "";
                break; // how to bring back to selection stage? 
            }
        }
        else{
            printf("You have selected: 'Venipuncture', please press again to confirm."); 
            msDelay(50); // wait for user to confirm
            if(BUTTON_PRESSED()){
                printf("'Venipuncture' Mode is confirmed."); 
                Mode = "VENIPUNCTURE";
                break;
            }
            else{
                printf("'Venipuncture' Mode NOT confirmed, please try again."); 
                Mode = "";
                break; // how to bring back to selection stage? 
            }
        }
    }
    return Mode; // output of function
} // end of UsageMode function

void TargetLimb { // Function 3: checking button for limb of usage
    printf("Please select Limb of Usage: press button for 'Bicep', or leave unpressed for 'Thigh'");
    msDelay(50); // wait for user to select mode
    char Limb[]; 
    while(1){
        if (BUTTON_PRESSED()){
            printf("You have selected: 'Thigh', please press again to confirm."); 
            msDelay(50); // wait for user to confirm
            if(BUTTON_PRESSED()){
                printf("'Thigh' Occlusion Mode is confirmed."); 
                Limb = "THIGH";
                TargetPressure = 300; // [mmHg]
                maxTime = 7200; // [s], or 2 [h]
                break;
            }
            else{
                printf("'Thigh' Occlusion Mode NOT confirmed, please try again."); 
                Limb = "";
                break; // how to bring back to selection stage? 
            }
        }
        else{
            printf("You have selected: 'Bicep', please press again to confirm."); 
            msDelay(50); // wait for user to confirm
            if(BUTTON_PRESSED()){
                printf("'Bicep' Occlusion Mode is confirmed."); 
                Limb = "BICEP";
                TargetPressure = 203; // [mmHg]
                maxTime = 5400; // [s], or 1.5 [h]
                break;
            }
            else{
                printf("'Bicep' Occlusion Mode NOT confirmed, please try again."); 
                Limb = "";
                break; // how to bring back to selection stage? 
            }
        }
    }
    return Limb, TargetPressure, maxTime; // outputs of function
} // end of TargetLimb function

// int detectPressure(){
//     // wait for the current reading to finish
//   while (digitalRead(2)) {}

//   // read 24 bits
//   long result = 0;
//   for (int i = 0; i < 24; i++) {
//     digitalWrite(3, HIGH);
//     digitalWrite(3, LOW);
//     result = result << 1;
//     if (digitalRead(2)) {
//       result++;
//     }
//   }

//   // get the 2s compliment
//   result = result ^ 0x800000;

//   // pulse the clock line 3 times to start the next pressure reading
//   for (char i = 0; i < 3; i++) {
//     digitalWrite(3, HIGH);
//     digitalWrite(3, LOW);
//   }
//     // display pressure
//   Serial.println(result);
//   return result;
// }


void setup() 
{ 
  pinMode(motorPin, OUTPUT);
  pinMode(2, INPUT);   // Connect HX710 OUT to Arduino pin 2
  pinMode(3, OUTPUT);  // Connect HX710 SCK to Arduino pin 3
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Speed 0 to 255");
} 
 
 
void loop() 
{   
  if (Serial.available())
  {
    int speed = Serial.parseInt();
    if (speed >= 0 && speed <= 255)
    {
      int pressure = 5;
      while(pressure != 4){
        analogWrite(motorPin, speed);
        pressure = pressure_sensor.mmHg();
        Serial.println(pressure);
      }
    }
  }
}
