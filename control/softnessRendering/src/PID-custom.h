#ifndef PID_custom
#define PID_custom

class PID
{
  public:
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
  PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double,  unsigned long, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)
	
  void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

  bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

  void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application

  void SetTunings(double, double, double);        	    

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.

  void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                        //   the PID calculation is performed.  default is 100
										  
  // METE edition
  void IntegratorAntiWindUpLimits(double Min, double Max); // * anti windup thing for pid. not necessary should be equal to the max PID output
	
  double integrator = 0;   	            // integrator
  double dInput = 0, dInputFiltered = 0;  // input velocity and filtered vel. can be monitored as well
  double myError = 0;                           // difference between output and input

  //Display functions ****************************************************************

  double LowpassFilter(double , double , double , double );

  private:
	void Initialize();          // why?
    
	double kp;                  // * (P)roportional Tuning Parameter
  double ki;                  // * (I)ntegral Tuning Parameter
  double kd;                  // * (D)erivative Tuning Parameter

  unsigned long SampleTime = 1;  // [ms]

	int controllerDirection;

  double *myInput;                // * Pointers to the Input, Output, and Setpoint variables
  double lastInput = 0;
  double *myOutput;               //   This creates a hard link between the variables and the 
  double *mySetpoint;             //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
                
	unsigned long lastTime;
  double outMin, outMax;
  double outMinIntegrator, outMaxIntegrator;

  double lowpassFilter(double, double, double);
  double filterBeta = 0.9;              // coeff
};
#endif