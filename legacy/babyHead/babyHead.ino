//babyHead reads information from an Accelerometer
// to write to the positions of two (Pan & Tilt) Servo Motors,
// imitating the orientation of the Accelerometer

//i.e. Mounting the Accelerometer on a Headset would
// make the Servos mimic the wearer's head orientation

#include <Servo.h>

Servo panServo;
const int panPin = 5;
Servo tiltServo;
const int tiltPin = 6;

const int xAccelPin = A0;   //For measuring Linear Acceleration along X-Axis
const int yAccelPin = A2;   //For measuring Linear Acceleration along Y-Axis
const int zAccelPin = A4;  //For measuring Linear Acceleration along Z-Axis
float xAccelOrigin, yAccelOrigin, zAccelOrigin; //Stores 'Original Values' of the 'Calibrated' Accelerometer
float xAngle, yAngle, zAngle = 0;
double sensitivity = 1;

void setup() {
  //Init Serial Comms
  Serial.begin(9600);
  
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);

  xAccelOrigin = analogRead(xAccelPin);
  yAccelOrigin = analogRead(yAccelPin);
  zAccelOrigin = analogRead(zAccelPin);

  Serial.println("Initialized");
}

void loop() {
  // put your main code here, to run repeatedly:

  xAngle = 0;
  for(int i = 0; i < 5; i++){
    xAngle += findAngle("PITCH");
  }
  xAngle /= 5; //Average it out for noise
 // yAngle = findAngle("YAW");
  
  Serial.print(xAngle);
  Serial.print("\t");
  Serial.print(yAngle);
  Serial.println();
  delay(1500);
}


//------------------------------------------------------------------------------------
//
//  Finds angular displacement from Two-Axis Differentials using 
//  Trigonometric Relationships; Returns in Degrees
//
//------------------------------------------------------------------------------------

float findAngle(String rotationalAxis){

  double accelAxisOne, accelAxisTwo; //Measure the changes along the two axes in question
  double xAccel = analogRead(xAccelPin);
  xAccel -= xAccelOrigin;
  double yAccel = analogRead(yAccelPin);
  yAccel -= yAccelOrigin;
  double zAccel = analogRead(zAccelPin);
  zAccel -= zAccelOrigin;

  Serial.print(xAccel);
  Serial.print("\t");
  Serial.print(yAccel);
  Serial.print("\t");
  Serial.print(zAccel);
  Serial.println();
  
  float finalAngle;

  if(rotationalAxis == "YAW"){
    //Intersection of Z & Y axes
    accelAxisOne = zAccel;
    accelAxisTwo = yAccel;
    //accelAxisTwo = sqrt(square(xAccel) + square(yAccel));
    
  } else if(rotationalAxis == "ROLL"){
    //Intersection of X & Y axes
    accelAxisOne = xAccel;
    accelAxisTwo = yAccel;
    //accelAxisTwo = sqrt(square(yAccel) + square(zAccel)); //More accurate rendering of dY, using all axes

  } else if (rotationalAxis == "PITCH"){
    //Intersection of Z & X axes
    accelAxisOne = zAccel;
    accelAxisTwo = xAccel;
    //accelAxisTwo = sqrt(square(xAccel) + square(yAccel)); //More accurate rendering of dX, using all axes

  }

  //By knowing the difference along one axis (e.g. dX) and another (e.g. dY),
  // computing the inverseTangent gives us the degree differential

  //if(accelAxisOne != accelAxisTwo){ //To not get noise
    finalAngle = atan2(accelAxisOne, accelAxisTwo) * (180/3.141592654); //Find angle and convert to Degrees for easier use with Servos
  //} else {
    //finalAngle = 0;
  //}
  

 // xAngle += finalAngle;
 // return finalAngle;
  return sin(accelAxisOne) * 180 / 3.1415;
}
