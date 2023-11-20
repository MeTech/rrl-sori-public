#include "main.h"

// Timing control
Timer Timer_01, Timer_02, Timer_03;

// pres_sens_pin, valve1_pin, valve2_pin, id
Pouch pouch1(A0, 0, 1, 1);
Pouch pouch2(A1, 2, 3, 2);

int fsrPin = A2, fsrValue=0;

int bendingSensorPin = A3, bendingSensorValue=0;
double origamiHeight = 0; // [mm]
double fsrResitance = 0, fsrVolDivRes=2600; //[ohm]
double fsrVoltage = 0, fsrVsupp = 3.3; // [v]
double fsrForce = 0, fsrForceFiltered = 0; // [N] 
double fsrOffset = 0; // [N]

int pinPotentiometer = A14, pinPotentiometer1 = A4;

double targetStiffness = 0.6; // [min-max] (0.57-2.42)
double targetStiffnessFiltered=0;
double targetOrigamiHeight = 7; // [mm] (min-max) (5-8)
double targetContactAreaRatio = 1.5; // [kPa/N] [0-2] // 9N max force

// EF30, EF50, DS10, EF50-DS30, DS30-EF50,
int userID=0, materialID=0;

double sinWaveValue = 0;
double sineWaveGenerator(double amplitude, double frequency, double timeSec, double offset){ 
  return amplitude*sin(2*PI*frequency*timeSec) + offset;
}
double amplitude = 7.5, frequency = 0.1;
double startTime, nowTime;


double fsrModel (double x, double offset){ // takes analog reading [0-1023] in and gives force [N] output. The voltage divider resistance 2.66 kOhm
  double modelResult = 1e-10*pow(x,4) - 9e-08*pow(x,3) + 3e-05*pow(x,2) + 0.0033*x + 0.0068; 
  double finalResult = modelResult - offset;
  if(finalResult<0)
    return 0;
  else
    return finalResult;
}

double bendingSensorModel(double x){ // input: analog output of the sensor [0-1023]
  return 0.0284*x - 0.0909;
}

double minStiffness = 0.3, maxStiffness = 6; // [N/mm] // min-max safe stiffness range of the device
  return 55.717*targetStiffness - 25.889; // Target Pressure
double stiffnessToPressureModelSDFeedforward(double targetStiffness){ 
}

double lowpassFilter(double previousFilteredValue, double input, double  beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}

// stiffness control
double forceSetpoint = 20 ; // [N] target force value
double forceControlOutput = 0;

double targetPressureFFW=0, targetPressureFB=0, targetPressureTotal=0, targetPressureTotalFiltered=0;
// Force PID variables and objectsx
double kP_FC=0, kI_FC=0, kD_FC=0;
// force PIDs
PID forceControlPID(&fsrForce, &forceControlOutput, &forceSetpoint,
kP_FC, kI_FC, kD_FC, FORCE_CONTROL_PERIOD, DIRECT);

void setup() {
  //SET analog write resolution to 12 bits [0-4095]
  analogWriteResolution(12);

  // to control the position of the arm
  pinMode(pinPotentiometer,INPUT);
  pinMode(pinPotentiometer1,INPUT);
  pinMode(fsrPin,INPUT);
  
  //  PRESSURE CONTROL
  // sets the input, output, and frequencies of the pins
  pouch1.setupPINs();
  pouch2.setupPINs();
  
  // initialize the setpoints
  pouch1.setpointPouch1Pressure = 0; // [kPa]
  pouch2.setpointPouch1Pressure = 0; // [kPa]

  pouch1.setupPIDs();
  pouch2.setupPIDs();
  
  double kP1= 40, kI1 = 50, kD1 = 0.1; // latest working one.
  double kP2= 60, kI2 = 500, kD2 = 0.7;
  
  pouch1.pouch1PID->SetTunings(kP1, kI1, kD1); // p,i,d
  pouch2.pouch1PID->SetTunings(kP2, kI2, kD2); // p,i,d

  //  FORCE CONTROL
  // FORCE PIDs
  double max_pressure = MAX_PRESSURE, min_pressure = MIN_PRESSURE; // [kPa]
  forceControlPID.SetOutputLimits(-max_pressure,max_pressure);
  forceControlPID.IntegratorAntiWindUpLimits(-max_pressure, max_pressure); // tune this

  kP_FC= 6, kI_FC= 5, kD_FC=0.05;
  forceControlPID.SetTunings(kP_FC, kI_FC, kD_FC);
  // I think i need feedforward

  // find fsrOffset value
  int avgSize = 10;
  double fsrForceSum = 0;
  //double pouch1PressureSum=0,pouch2PressureSum=0;
  // double pouch1PressureOffset=0,pouch2PressureOffset=0;
  for(int i=0; i<avgSize;i++){
    fsrValue = analogRead(fsrPin);
    fsrForceSum += fsrModel(fsrValue,0); // [N] 
    // pouch1PressureSum += pouch1.readPressure(); 
    // pouch2PressureSum += pouch2.readPressure();
  }
  fsrOffset = fsrForceSum/avgSize;
  // pouch1PressureOffset = pouch1PressureSum/avgSize;
  // pouch2PressureOffset = pouch2PressureSum/avgSize;
  

  // initialize the times
  Timer_01.initialize(PRESSURE_CONTROL_PERIOD);
  Timer_02.initialize(SERIAL_PRINT_PERIOD);
  Timer_03.initialize(FORCE_CONTROL_PERIOD);

	Serial.begin(BAUD_RATE); // update in .ini file as well
  startTime = millis();
}

void loop() {
  // ---- Pressure Control --- @pressureControlLoop[1ms]
  if(Timer_01.Tick()){
    // Force sensor read    
    fsrValue = analogRead(fsrPin);
    // ToDos: Need to check the offset at the beginning!
    fsrForce = fsrModel(fsrValue, fsrOffset); // [N] 
    fsrForceFiltered = lowpassFilter(fsrForceFiltered, fsrForce, 0.9);

    // origami height read
    bendingSensorValue = analogRead(bendingSensorPin);
    // ToDos: calibrate properly!
    origamiHeight =  bendingSensorModel((double)bendingSensorValue);

    // potentiometer controlled stiffness
    // targetStiffness = minStiffness + (maxStiffness-minStiffness) * analogRead(pinPotentiometer)/1023.0;
    
    targetOrigamiHeight = 9; // [mm]
    double dispOrigami = targetOrigamiHeight-origamiHeight;
    // hand assignes stiffness!
    //targetStiffness = 5; // [N/mm] [0.65-2.3]
    //forceSetpoint = (dispOrigami)*targetStiffness;

    forceSetpoint = softnessRenderingStiffnessModel(userID, materialID, dispOrigami); // userID, materialID, displacement

    // RUN FORCE CONTROLLER -- outout is feedback pressure
    if(Timer_03.Tick()){
      forceControlPID.Compute();
    }

    targetPressureFB = forceControlOutput;
    // this part is wrong!!! need to fix it!
    //  // feedforward terms!
    if(dispOrigami>0.3)
      targetStiffness = forceSetpoint/dispOrigami;
    else
      targetStiffness = 0;

    targetStiffnessFiltered = lowpassFilter(targetStiffnessFiltered, targetStiffness, 0.98);

    targetPressureFFW = stiffnessToPressureModelSDFeedforward(targetStiffness); 

    if(targetPressureFFW > MAX_PRESSURE)
      targetPressureFFW = MAX_PRESSURE;
    else if (targetPressureFFW < MIN_PRESSURE)
      targetPressureFFW = 0;
    targetPressureTotal = targetPressureFB + targetPressureFFW;
    
    if(targetPressureTotal > MAX_PRESSURE)
      targetPressureTotal = MAX_PRESSURE;
    else if (targetPressureTotal < MIN_PRESSURE)
      targetPressureTotal = 0;
    
    targetPressureTotalFiltered= lowpassFilter(targetPressureTotalFiltered, targetPressureTotal, 0.90);

    // Control input
    //pouch1.setpointPouch1Pressure = targetPressureTotalFiltered;

    // LETS first test the pressure control!
    pouch1.setpointPouch1Pressure =  90 * analogRead(pinPotentiometer)/1023.0;
    
    // Contact Area Rendering
    //fsr
    nowTime = millis() -startTime;
    amplitude = 7.5, frequency = 96.0;
    
    sinWaveValue = sineWaveGenerator(amplitude, frequency, nowTime/1000.0, amplitude+5);

  
    // targetContactAreaRatio = 8*(1.0-analogRead(pinPotentiometer)/1023.0);
    //pouch2.setpointPouch1Pressure = fsrForceFiltered * targetContactAreaRatio; //0; //15 * analogRead(pinPotentiometer)/1023.0;

    // // for bandwidth analysis
    // pouch2.setpointPouch1Pressure = sinWaveValue;
    
    double softnessCoeff = 10;
    pouch2.setpointPouch1Pressure =  softnessCoeff * fsrForceFiltered * analogRead(pinPotentiometer1)/1023.0;

    // Final control input
    //pouch2.setpointPouch1Pressure = softnessRenderingContactAreaModel(userID, materialID, fsrForceFiltered); // userID, materialID, force
    
    if(pouch2.setpointPouch1Pressure>24){ // kPa
      pouch2.setpointPouch1Pressure = 24; // kPa
    }

    pouch1.readPressure();
    pouch2.readPressure();

    // RUN THE PRESSURE CONTROLLER (PID)
    pouch1.computePressureController();  // updates the desired PWM outputs
    pouch2.computePressureController();  // updates the desired PWM outputs
 
    // GET THE PID OUTPUT - PWM OUTPUTS
    pouch1.computePWMSetpoints(); // calculates the desired PWM setpoints
    pouch2.computePWMSetpoints(); // calculates the desired PWM setpoints

    // SET PWM VALUES
    pouch1.setPWMs();
    pouch2.setPWMs();

   //if(nowTime< (5*1000/frequency)){
    if(Timer_02.Tick()){ // Print to serial
      /*
      Serial.print(pouch1.setpointPouch1Pressure);Serial.print("\t");
      Serial.print(pouch1.pouch1Pressure);Serial.print("\t");

      Serial.print(pouch2.setpointPouch1Pressure);Serial.print("\t");
      Serial.print(pouch2.pouch1Pressure);Serial.print("\t");  
      
      Serial.print(targetOrigamiHeight);Serial.print("\t");
      Serial.print(origamiHeight);Serial.print("\t");

      Serial.print(forceSetpoint);Serial.print("\t");
      Serial.print(fsrForceFiltered);Serial.print("\t");
      */
      
      // Serial.print(pouch1.pwmOutput1);Serial.print("\t");
      // Serial.print(pouch1.pwmOutput2);Serial.print("\t");
      
      //Serial.print(bendingSensorValue);Serial.print("\t");
      
      //Serial.print(fsrForce);Serial.print("\t");
      Serial.print(analogRead(pinPotentiometer));Serial.print("\t");
      Serial.print(analogRead(pinPotentiometer1));Serial.print("\t");
      
      Serial.println();
    }
    //}
  }

}
