/**********************************************************************************************
* Pouch Motor Control
* by Mustafa Mete <mustafa.mete@epfl.ch>
**********************************************************************************************/
#ifndef PouchMotor
#define PouchMotor

#include "PID-custom.h"
#define MIN_PRESSURE          0                 // [kPa]
#define MAX_PRESSURE          100                // [kPa]
#define MAX_PWM               4095
#define MIN_PWM               2700

// Desired sampling time or period of the loop in miliseconds
#define PRESSURE_CONTROL_PERIOD 1 // [ms]
#define FORCE_CONTROL_PERIOD 10 // [ms]
#define SERIAL_PRINT_PERIOD 100 // [ms]

// Teensy setup parameters
#define PWM_FREQUENCY               10000 // 36621.09    // [Hz] suggested fro 12 bit resolution
#define PWM_RESOLUTION              12          // [bits]
#define BAUD_RATE                   57600       

class Pouch
{
    private:
    public:
    // Pins for pressure sensors
    int presSenPIN1;

    // Pins for valves
    int valvePIN1, valvePIN2;

    // the id is necceessary for calibration of different hardware (pres. sens i.e.)
    int pouchID;

    // Constructor  for 3 DoF
    Pouch(int, int, int, int);
    
    double pouch1Pressure = 0, setpointPouch1Pressure = 0;
    int  setpointPWM1 = 0, setpointPWM2 = 0;

    // assign the input and output pins and sets the frequencies
    void setupPINs();
   
    //sub methods
    double convertPressureSensorVoltageToKpa(double);

    // Pouch PID variables and objects
    double pouch1PIDkP=0, pouch1PIDkI=0, pouch1PIDkD=0;

    // PID outputvariables
    double pwmOutput1=0, pwmOutput2=0;

    // pouch PIDs
    PID* pouch1PID;

    void readPressure(); // method to get pressure
    void setupPIDs();
    void computePressureController();
    void computePWMSetpoints();
    void setPWMs();
    double lowpassFilter(double, double, double);
    bool isTheSignSame(double, double);// returns  TRUE if they have the same sign, else false
};

#endif