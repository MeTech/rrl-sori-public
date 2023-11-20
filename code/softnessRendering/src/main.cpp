#include "main.h"

// Timing control
Timer Timer_01, Timer_02, Timer_03;

// pres_sens_pin, valve1_pin, valve2_pin, id
Pouch pouch1(A0, 0, 1, 1);
Pouch pouch2(A1, 2, 3, 2);



double sinWaveValue = 0;
double sineWaveGenerator(double amplitude, double frequency, double timeSec, double offset){ 
  return amplitude*sin(2*PI*frequency*timeSec) + offset;
}
double heartBeatGenerator(double amplitude, double frequency, double timeSec, double offset){
  double period = 1/frequency;
  double deltaTime = fmod(timeSec,period);
  double value = 0;
  if(deltaTime<period/4)
    value = amplitude*sin(2*PI*2*frequency*deltaTime) + offset;
  else if(deltaTime<period/2)
    value = 0.5*amplitude*sin(2*PI*2*frequency*deltaTime) + offset;
  else
    value = 0.5*amplitude*sin(2*PI*2*frequency*period/2) + offset;
  return value;
}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

double fsrModel (double x, double offset){ // takes analog reading [0-1023] in and gives force [N] output. The voltage divider resistance 2.66 kOhm
  double modelResult = 1e-10*pow(x,4) - 9e-08*pow(x,3) + 3e-05*pow(x,2) + 0.0033*x + 0.0068; 
  double finalResult = modelResult - offset;
  if(finalResult<0)
    return 0;
  else
    return finalResult;
}

// changes between 8-13 mm
double bendingSensorModel(double x){ // input: analog output of the sensor [0-1023]
  return 8E-07*pow(x,3) - 0.0005*pow(x,2) + 0.1189*x - 0.1103; 
}

double stiffnessToPressureModelSDFeedforward(double targetStiffness){ // min-max stiffness = [0.57-2.42] // gives the stiffness of the device
  return 55.717*targetStiffness - 25.889; // Target Pressure
}

double lowpassFilter(double previousFilteredValue, double input, double  beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}

// force PIDs
PID forceControlPID(&fsrForce, &forceControlOutput, &forceSetpoint,
kP_FC, kI_FC, kD_FC, FORCE_CONTROL_PERIOD, DIRECT);

void setup() {
  //SET analog write resolution to 12 bits [0-4095]
  analogWriteResolution(12);

  pinMode(pinPotentiometer,INPUT);
  pinMode(pinPotentiometer1,INPUT);
  pinMode(fsrPin,INPUT);
  
  //  PRESSURE CONTROL
  // sets the input,output, and frequencies of the pins
  pouch1.setupPINs();
  pouch2.setupPINs();
  
  // initialize the setpoints
  pouch1.setpointPouch1Pressure = 0; // [kPa]
  pouch2.setpointPouch1Pressure = 0; // [kPa]

  pouch1.setupPIDs();
  pouch2.setupPIDs();

  // tunings 
  //pressure pid parameters
  //stiffness
  double kP1= 30, kI1 = 100, kD1 = 0.1; // latest working one.
  //contact area
  double kP2= 60, kI2 = 500, kD2 = 0.7;
  // force pid parameters
  //kP_FC= 6, kI_FC= 15, kD_FC=0.05;
  kP_FC= 10, kI_FC= 50, kD_FC=0.05;

  // stiffness PID
  pouch1.pouch1PID->SetTunings(kP1, kI1, kD1); // p,i,d
  //contact area PID
  pouch2.pouch1PID->SetTunings(kP2, kI2, kD2); // p,i,d

  //  ----------  FORCE CONTROL------------
  double max_pressure = MAX_PRESSURE; // min_pressure = MIN_PRESSURE; // [kPa]
  forceControlPID.SetOutputLimits(-max_pressure,max_pressure);
  forceControlPID.IntegratorAntiWindUpLimits(-max_pressure, max_pressure); // tune this
  forceControlPID.SetTunings(kP_FC, kI_FC, kD_FC);

  // find fsrOffset value
  int avgSize = 10;
  double fsrForceSum = 0;
  for(int i=0; i<avgSize;i++){
    fsrValue = analogRead(fsrPin);
    fsrForceSum += fsrModel(fsrValue,0);
  }
  fsrOffset = fsrForceSum/avgSize;  

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
    while (Serial.available()>0) {
        char inChar = (char)Serial.read();
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a newline, set a flag so the main loop can
        // do something about it:
        if (inChar == '\n') {
          stringComplete = true;
        }
    }
    if(stringComplete){ 
      vrMode = getValue(inputString, ',', 0).toFloat(); 
      inputString = "";
      stringComplete = false;
      Serial.flush();
    }  

    if(vrMode == 0){ // soft soft
      vrTargetPressure1=0.1; vrTargetPressure2=0.7;
    }
    else if(vrMode == 1){ // hard hard
      vrTargetPressure1=0.9; vrTargetPressure2=0.1;
    }
    else if(vrMode == 2){ // hard soft
      vrTargetPressure1=0.9; vrTargetPressure2=0.3;
    }
    else if(vrMode == 3){ // soft hard
      vrTargetPressure1=0.2; vrTargetPressure2=0.0;
    }
    else {
      vrTargetPressure1=0; vrTargetPressure2=0;
    }


    // Force sensor read    
    fsrValue = analogRead(fsrPin);
    // ToDos: Need to check the offset at the beginning!
    fsrForce = fsrModel(fsrValue, fsrOffset); // [N] 
    fsrForceFiltered = lowpassFilter(fsrForceFiltered, fsrForce, 0.9);

    // origami height read
    bendingSensorValue = analogRead(bendingSensorPin);
 
    origamiHeight =  bendingSensorModel((double)bendingSensorValue);

    // ------- Stiffness Rendering ----------
    // potentiometer controlled stiffness
    // targetStiffness = minStiffness + (maxStiffness-minStiffness) * analogRead(pinPotentiometer)/1023.0;
    
    // ------- VR Controlled -------
    //targetStiffness = minStiffness + (maxStiffness-minStiffness) * vrTargetPressure1;
    pouch1.setpointPouch1Pressure =  100* vrTargetPressure1;
    
    // -------  Heartbeat ------- 
    // amplitude = 6,frequency = 0.5;
    // nowTime = millis() -startTime;
    // double heartBeatDeltaHeight = heartBeatGenerator(amplitude, frequency, nowTime/1000.0, 0);
    // targetOrigamiHeight = 20+heartBeatDeltaHeight; // [mm]
    
    // -------  Hand Assigned ------- 
    double dispOrigami = targetOrigamiHeight-origamiHeight;
    //targetStiffness = 0.6; // [N/mm] [0.65-2.3]
    forceSetpoint = (dispOrigami)*targetStiffness;

    // ------- Material based ------- 
    // forceSetpoint = softnessRenderingStiffnessModel(userID, materialID, dispOrigami); // userID, materialID, displacement

    // RUN FORCE CONTROLLER -- outout is feedback pressure
    if(Timer_03.Tick()){
      forceControlPID.Compute();
    }
    targetPressureFB = forceControlOutput;
   
    // feedforward terms!
    /**/
    if(dispOrigami>0.3)
      targetStiffness = forceSetpoint/dispOrigami;
    else
      targetStiffness = 0;

    targetStiffnessFiltered = lowpassFilter(targetStiffnessFiltered, targetStiffness, 0.98);
    targetPressureFFW = stiffnessToPressureModelSDFeedforward(targetStiffness); 
    

    targetPressureTotal = targetPressureFB + targetPressureFFW;
    // limit the target
    if(targetPressureTotal > MAX_PRESSURE)
      targetPressureTotal = MAX_PRESSURE;
    else if (targetPressureTotal < MIN_PRESSURE)
      targetPressureTotal = 0;
    
    targetPressureTotalFiltered= lowpassFilter(targetPressureTotalFiltered, targetPressureTotal, 0.90);

    // Stiffness control pressure input setpoint
    pouch1.setpointPouch1Pressure = targetPressureTotalFiltered;

    // Potentiometer controlled pressure setpoint
    // pouch1.setpointPouch1Pressure =  90 * analogRead(pinPotentiometer)/1023.0;
    

    // Contact Area Rendering

    /*
    // for bandwidth analysis
     nowTime = millis() -startTime;
     amplitude = 7.5, frequency = 96.0;
     sinWaveValue = sineWaveGenerator(amplitude, frequency, nowTime/1000.0, amplitude+5);
     pouch2.setpointPouch1Pressure = sinWaveValue;
    */
  
    //for manual control
    //pouch2.setpointPouch1Pressure =  softnessCoeff * fsrForceFiltered * analogRead(pinPotentiometer1)/1023.0;

    //for vr control
    pouch2.setpointPouch1Pressure =  softnessCoeff * fsrForceFiltered * vrTargetPressure2;

    //pouch2.setpointPouch1Pressure =  softnessCoeff * fsrForceFiltered;

    // for heart experiment!
    //softnessCoeff = 2;
    //pouch2.setpointPouch1Pressure =  softnessCoeff * fsrForceFiltered * fsrForceFiltered * fsrForceFiltered* analogRead(pinPotentiometer1)/1023.0;

    // for material tests!
    //pouch2.setpointPouch1Pressure = softnessRenderingContactAreaModel(userID, materialID, fsrForceFiltered); // userID, materialID, force
    
    if(pouch2.setpointPouch1Pressure > 24){ // kPa
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

    if(Timer_02.Tick()){ // Print to serial
      Serial.print(pouch1.setpointPouch1Pressure);Serial.print("\t");
      Serial.print(pouch1.pouch1Pressure);Serial.print("\t");  

      Serial.print(pouch2.setpointPouch1Pressure);Serial.print("\t");
      Serial.print(pouch2.pouch1Pressure);Serial.print("\t");  
      
      Serial.print(targetOrigamiHeight);Serial.print("\t");
      Serial.print(origamiHeight);Serial.print("\t");

      Serial.print(forceSetpoint);Serial.print("\t");
      Serial.print(fsrForceFiltered);Serial.print("\t");

      Serial.println();
    }
  }

}
