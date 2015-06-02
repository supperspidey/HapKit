//--------------------------------------------------------------------------
// Original Authors:
// Tania Morimoto and Allison Okamura, Stanford University
// 11.16.13
// Code to test basic Hapkit functionality (sensing and force output)
//--------------------------------------------------------------------------
// Modified for Hapkit Rowing Simulation by Van Le Nguyen, Ilhaan Rasheed and Bryce Walburn
// ECES-490/690, Drexel University, Spring 2015
//--------------------------------------------------------------------------

// Includes
#include <math.h>

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

// Kinematics variables
double xh = 0;           // position of the handle [m]
double lastXh = 0;     //last x position of the handle
double vh = 0;         //velocity of the handle
double lastVh = 0;     //last velocity of the handle
double lastLastVh = 0; //last last velocity of the handle

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

// Angle and time variables
double oldOarAngle = 0;
double currentOarAngle = 0;
unsigned long oldTimeSinceBegin = 0;
unsigned long currentTimeSinceBegin = 0;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup()
{
  // Set up serial communication
  Serial.begin(57600);

  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A

  // Initialize motor
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  currentTimeSinceBegin = millis();

  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if ((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (lastRawDiff > 0) {       // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    if (rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber * rawOffset; // update the pos value to account for flips over 180deg using the most current offset
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber * lastRawOffset; // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPos = rawPos + flipNumber * tempOffset; // need to update pos based on what most recent offset is
    flipped = false;
  }

  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  double rh = 0.065659;   //[m]
  // Step 2.1: print updatedPos via serial monitor
  //Serial.println(updatedPos);
  // Step 2.6:
  double ts = -.0107 * updatedPos + 4.9513; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos

  // Step 2.7:
  xh = rh * (ts * 3.14159 / 180); // Compute the position of the handle based on ts
  // Step 2.8: print xh via serial monitor
  //Serial.println(xh,5);
  // Lab 4 Step 2.3: compute handle velocity
  vh = -(.95 * .95) * lastLastVh + 2 * .95 * lastVh + (1 - .95) * (1 - .95) * (xh - lastXh) / .0001; // filtered velocity (2nd-order filter)
  lastXh = xh;
  lastLastVh = lastVh;
  lastVh = vh;

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  double rp = 0.004191;   //[m]
  double rs = 0.073152;   //[m]

  // Rowing code
        double A = 0.146;                                // Area of the blade [m^2]
        double rho = 1000;                               // Density of water  [kg/m^3]
        double vb = 5;                                    // Velocity of the boat [m/s]
        double L = 1.8;                                 // Travel length of the rowing seat [m]
        double angleRadians = ts*3.14159/180;                      // Angle converted into radians
        currentOarAngle = angleRadians;
        double Clmax = 1.2;  // Maximum lift coefficient
        double dphidt = (currentOarAngle - oldOarAngle) /  ((currentTimeSinceBegin - oldTimeSinceBegin)/1000.0); // Derivative of the oar angle

        double up = vb*sin(angleRadians);
        double ul = dphidt*L - vb*cos(angleRadians);
        double alpha = atan2(ul, up);

        double Cd = 2*Clmax*pow(sin(alpha), 2);          // Drag coefficient
        double Cl = Clmax*sin(2*alpha);

        double v = sqrt(pow(up, 2) + pow(ul, 2));
        double Fd = 0.5*Cd*rho*A*pow(v, 2);
        double Fl = 0.5*Cl*rho*A*pow(v, 2);

        // Air - coefficient and force
        double rhoAir = 1.225;  // Density of air  [kg/m^3]
        double CdAir = 0.04;    // Drag coefficient of streamlined body
        double FdAir = 0.5*CdAir*rhoAir*A*pow(vh, 2);

        double scale = 0.001;

        // Calculate differnce between currentOarAngle and oldOarAngle
        double difference = currentOarAngle - oldOarAngle;

        if (difference < 0) {
          Serial.print("Air");
          Serial.print(": ");
          Serial.print(currentOarAngle);
          Serial.print(", ");
          //force = FdAir; // air
          Serial.print(force);
        } else if (difference > 0) {
          Serial.print("Water");
          Serial.print(": ");
          Serial.print(currentOarAngle);
          Serial.print(", ");
          //force = sqrt(pow(Fd, 2) + pow(Fl, 2)) * cos(currentOarAngle) * scale; // water
          //force = 5 * (-3.8*pow(currentOarAngle, 2) - 3.7529*currentOarAngle + 0.0241);
          Serial.print(force);
        } else {
          Serial.print("Stop");
          Serial.print(": ");
          Serial.print(currentOarAngle);
          Serial.print(", ");
          force = 0;
          Serial.print(force);
        }

  Serial.println();

  // Step 3.2:
  Tp = rp / rs * rh * force;  // Compute the require motor pulley torque (Tp) to generate that force


  //This next section is OPTIONAL depending on if you have a FSR
  //*************************************************************
  //*** Section 4. Compute measured handle force in Newtons *****
  //*************************************************************

  // ADD YOUR CODE HERE
  // Step 4.1:
  int fsrValue = analogRead(fsrPin);  // Read the analog input value from the FSR pin ansd store in the variable fsrValue
  // Step 4.2: print fsrValue via serial monitor
  //Serial.println(fsrValue);
  // Step 4.7:
  float fsrForce = -.0026 * fsrValue; // Compute the (approximate) FSR measured force value in Newtons, based on calibration
  // Step 4.8: print fsrForce via serial monitor
  //Serial.println(fsrForce);
  // Lab 4 Step 1.10:
  //Serial.print(xh,5);        //print xh (in m), without a line break
  //Serial.print("\t");          //print a tab, without a line break
  //Serial.println(fsrForce);  //print the measured fsrForce (in N), with a line break
  // Lab 4 Step 2.7: record virtual damper data
  //Serial.print(vh,5);        //print xh (in m), without a line break
  //Serial.print("\t");          //print a tab, without a line break
  //Serial.println(fsrForce);  //print the measured fsrForce (in N), with a line break
  //*************************************************************
  //*** Section 5. Force output (do not change) *****************
  //*************************************************************

  // Determine correct direction for motor torque
  if (force < 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal

  oldOarAngle = currentOarAngle;
  oldTimeSinceBegin = currentTimeSinceBegin;
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
