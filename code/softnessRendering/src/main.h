#include <Arduino.h>
#include "Timing.cpp"           // Custom class to implement accurate timing functions
#include "pouch.h"
#include "softnessRendering.h"


double minPressure = MIN_PRESSURE, maxPressure = MAX_PRESSURE; // [kPa]
unsigned long now, period;
// stiffness, surface
// 0-> soft, soft, 1-> hard, hard, 2-> hard, soft, 3-> soft, hard
int vrMode = 0; 
float vrTargetPressure1=0; // [0-1]
float vrTargetPressure2=0; // [0-1]


String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


int fsrPin = A2, fsrValue=0;
int bendingSensorPin = A3, bendingSensorValue=0;
double origamiHeight = 0; // [mm]
double fsrResitance = 0, fsrVolDivRes=2600; //[ohm]
double fsrVoltage = 0, fsrVsupp = 3.3; // [v]
double fsrForce = 0, fsrForceFiltered = 0; // [N] 
double fsrOffset = 0; // [N]

int pinPotentiometer = A14, pinPotentiometer1 = A4;

double softnessCoeff = 15;
double targetStiffness = 0.6; // [min-max] (0.57-2.42)
double targetStiffnessFiltered=0;
double targetOrigamiHeight = 22; // [mm] (min-max) (5-8)
double targetContactAreaRatio = 1.5; // [kPa/N] [0-2]

// EF30, EF50, DS10, EF50-DS30, DS30-EF50,
int userID=0, materialID=6;

double amplitude = 7.5, frequency = 0.01;
double startTime, nowTime;

double minStiffness = 0, maxStiffness = 6; // [N/mm]

// stiffness control
double forceSetpoint = 18 ; // [N] target force value
double forceControlOutput = 0;
double targetPressureFFW = 0, targetPressureFB = 0, targetPressureTotal = 0, targetPressureTotalFiltered = 0;
// Force PID variables and objectsx
double kP_FC=0, kI_FC=0, kD_FC=0;

