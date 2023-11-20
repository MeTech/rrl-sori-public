/**********************************************************************************************
 * Arduino PID Library
 * modified by Mustafa Mete <mustafa.mete@epfl.ch>
 **********************************************************************************************/
#include "Arduino.h"
#include "PID-custom.h"

//Constructor
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, unsigned long SampleTime, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint; 
    this->SampleTime = SampleTime;              //default Controller Sample Time is 0.01 seconds  // [ms]

    PID::SetOutputLimits(0, 255);				//default output limit corresponds to //the arduino pwm limits
    PID::IntegratorAntiWindUpLimits(0, 255);

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis()-SampleTime;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);

   if(timeChange>=SampleTime)
   {
      double input = *myInput;
         myError = *mySetpoint - input; // should error be global?
         //if(abs(myError)<0.5){ return true; }
         dInput = (input - lastInput);
         // some anti-windup - forgot the term.
         if ((integrator>0 && myError>0) && (integrator<0 && myError<0))
            integrator+= (ki * myError);
         else
            integrator+= 8*(ki * myError);

        // clamping the pid integrator
        if(integrator > outMaxIntegrator) integrator = outMaxIntegrator;
        else if(integrator < outMinIntegrator) integrator = outMinIntegrator;

        // filter the velocity

        filterBeta = 0.9;        
        dInputFiltered = this->lowpassFilter(dInputFiltered, dInput, filterBeta);

        double output;
        // PID
        output = kp * myError + integrator + kd * dInputFiltered;
        //clamping the pid output
        if(output > outMax) output = outMax;
        else if(output < outMin) output = outMin;

        *myOutput = output;

        // Remember some variables for next time 
        lastInput = input;
        lastTime = now;
        return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}



/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
}

void PID::IntegratorAntiWindUpLimits(double Min, double Max){
   if(Min >= Max) return;
   outMinIntegrator= Min;
   outMaxIntegrator= Max;
}


/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.) 
 ******************************************************************************/
void PID::SetControllerDirection(int Direction){
   if(Direction !=controllerDirection){
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
   }
   controllerDirection = Direction;
}

//change this to moving average
double PID::lowpassFilter(double previousFilteredValue, double input, double  beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}