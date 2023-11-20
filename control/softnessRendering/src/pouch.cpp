/**********************************************************************************************
* Pouch pressure control
* by Mustafa Mete <mustafa.mete@epfl.ch>
**********************************************************************************************/

#include "Arduino.h"
#include "pouch.h"

/* Constructor*/
Pouch::Pouch(int presSenPIN1, int valvePIN1, int valvePIN2, int pouchID){
      this->presSenPIN1 = presSenPIN1;
      this->valvePIN1 = valvePIN1; this->valvePIN2 = valvePIN2;
      this->pouchID = pouchID;
    }

void Pouch::setupPINs(){  // needs to be called after the creation of the objects
  // pressure sensor pins
  pinMode(this->presSenPIN1, INPUT);

  // pwm pins for valves
  pinMode(this->valvePIN1, OUTPUT);
  pinMode(this->valvePIN2, OUTPUT);

  analogWriteFrequency(this->valvePIN1, PWM_FREQUENCY); // suggested frequency
  analogWriteFrequency(this->valvePIN2, PWM_FREQUENCY); // suggested frequency
}

// pressure sensor model: // converts the input to voltage values
void Pouch::readPressure(){
  //pressureMeasuredUnfiltered // for the pressure sensor Honeywell 030PGAA5
  //double unfilteredmodulePouchPressure1 =  413.7*(analogRead(this->presSenPIN1)*(5.0/1023.0) - 0.5)/4 + 2.18; // 060PGAA5
  double max_pressure = 206.84; //kPa
  double  R1 = 0.47, R2 = 1; // kohm
  double Vsup = R2/(R1+R2)*5; //voltage divider;
  double Vout = analogRead(this->presSenPIN1) * (Vsup / 1023.0);

  // for the pressure sensor Honeywell ABPDAN 030pgaa5
  double unfilteredmodulePouchPressure1 = max_pressure/0.9 * (Vout/Vsup-0.1); // for pres sens 1!!
  double offset = 1.7; // kPa
  // filter the pressure
  if ((unfilteredmodulePouchPressure1-offset)<0)
    this->pouch1Pressure = 0;
  else
    this->pouch1Pressure = unfilteredmodulePouchPressure1-offset; // this->lowpassFilter(this->pouch1Pressure, unfilteredmodulePouchPressure1, 0.1);
}

// -------- SETPOINTS!! --------/////
void Pouch::computePWMSetpoints(){
  if(pwmOutput1>0){
    this->setpointPWM1 = (int) this->pwmOutput1 +  (int) MIN_PWM;
    this->setpointPWM2 = (int) MIN_PWM;
  }
  else{
    this->setpointPWM1 = (int) MIN_PWM;
    this->setpointPWM2 = (int) abs(this->pwmOutput1) + (int) MIN_PWM;
  }
}

void Pouch::setupPIDs(){
  // ----   POUCH PIDS ------//
  this->pouch1PID = new  PID(&(this->pouch1Pressure), &pwmOutput1, &setpointPouch1Pressure,
     pouch1PIDkP, pouch1PIDkI, pouch1PIDkD, PRESSURE_CONTROL_PERIOD, DIRECT); 
  this->pouch1PID->SetOutputLimits((double)MIN_PWM-MAX_PWM,(double)MAX_PWM-MIN_PWM);
  
  this->pouch1PID->IntegratorAntiWindUpLimits(-2000.0, 2000.0);
}

void Pouch::computePressureController(){
  this->pouch1PID->Compute();
}

// -------- ASSIGN PWM OUTPUTS TO PINS --------/////
void Pouch::setPWMs(){
  analogWrite(this->valvePIN1, this->setpointPWM1);
  analogWrite(this->valvePIN2, this->setpointPWM2);
}

double Pouch::lowpassFilter(double previousFilteredValue, double input, double  beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}

bool Pouch::isTheSignSame(double x, double y){
  return (x>0) == (y>0);
}