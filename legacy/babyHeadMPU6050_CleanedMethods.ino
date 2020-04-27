/*

  Interfaces with MPU6050 (reading Gyro & Accelerometer data) to 
  Operate a Pan & Tilt Servo Combination (to mimic the orientation
  of the IMU / MPU6050)

*/

//  Is it really that bad to comment every line of code?
//    I feel like it usually is, but--in this context, where
//    nigh' everything is new, I feel like it's needed

#include <Wire.h>     //The Wire Library allows an Arduino to communicate with I2C (like the MPU6050) and TWI Devices

const int MPU = 0x68; //MPU6050 I2C Address (Hex)

float AccX, AccY, AccZ;
float AccAngleX, AccAngleY;

float GyroX, GyroY, GyroZ;
float GyroAngleX, GyroAngleY, GyroAngleZ;
float currentTime, lastTime, dT = 0;

float bodyX, bodyY, bodyZ; //Stores the Pitch/Roll/Yaw Angles relative to the IMU Reference Frame

float inertialX = 0;  // inertialXYZ stores the Euler Angles relative to the 
float inertialY = 0;  //  Inertial Frame of reference (i.e. it contains the degrees 
float inertialZ = 0;  //  X, Y, & Z to rotate the Inertial Frame into the existing Body Frame)
float velInertX, velInertY, velInertZ; // Stores Rotational Velocity of the Inertial Frame's X, Y, Z axes

float panAngle;
float tiltAngle; //Necessary due to 'runaways' with converting inertialXYZ to Degrees on its own

float lockTolerance = 0.01; //  Defines the values at which we switch to alternative methods of angle acquisition to prevent Gimbal Lock (i.e. when the reference vector is rotating on the Z Axis)

void setup() {
    Serial.begin(9600);
    
    Wire.begin();                   //  Start Communicatin'
    Wire.beginTransmission(MPU);    //  Open a channel with the MPU (which is at register 0x6B)
    Wire.write(0x6B);               //  Talk to Register 0x6B, which contains the DEVICE_RESET Bit
    Wire.write(0x00);               //  Reset the MPU by writing in 0 (sets DEVICE_RESET to 0)
    Wire.endTransmission(true);
    
    Serial.println("Initialized");    
}

void loop() {
    
    readMPU();                    //  Read necessary data from the MPU
    calculateEulerAngles();       //  Calculate the values needed to orient the Pan/Tilt Servo
    
}

//-----------------------------------------------
//
//  Reads the MPU for Gyro & Accel Data
//
//-----------------------------------------------

void readMPU(){

    //  Read Accelerometer Data
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);               //  Start with Register 0x3B (Accel_Xout[Bits 15-8])
    Wire.endTransmission(false);    //  Allows sending multiple messages (Rather than ending the transmission, releasing the I2C Bus, we send a restart message)
    Wire.requestFrom(MPU, 6, true); //  Lets us read the 6 registers containing the Accel. readouts (each axis is stored in two bytes)
                                    //    Wire.requestFrom(int Address, int Quantity, bool Stop) reads from our previous Address, incrementing it, sending a restart message 
                                    //    (keeping the bus open until we've reached the end (i.e. until we've requested the address Quanitity times)
    AccX = ((Wire.read() << 8 | Wire.read()) / 16384.0) - 0.05; //To Explain later
    AccY = ((Wire.read() << 8 | Wire.read()) / 16384.0) - 0.03;
    AccZ = ((Wire.read() << 8 | Wire.read()) / 16384.0) + 0.005;
    
    //  Find Body-Frame Pitch & Roll
    AccAngleX = atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180/PI;
    AccAngleY = atan2(AccX, sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180/PI;
    
    //  Read Gyroscope Data
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x43);             //  Start with Register 0x43 (Gyro_Xout[Bits 15-8])
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    GyroX = ((Wire.read() << 8 | Wire.read()) - 1190.8) / 131.0; //  Given in deg/s
    GyroY = ((Wire.read() << 8 | Wire.read()) - 132.5)  / 131.0; // /90.0
    GyroZ = ((Wire.read() << 8 | Wire.read()) + 297.77) / 131.0;
    
    currentTime = millis();                         //  Find Current Time
    dT = (currentTime - lastTime) / 1000;           //  Find Elapsed Time, then convert to Seconds
    lastTime = currentTime;                         //  Reset Time for next data point
    
    GyroAngleX += GyroX * dT;     //  DegreesX = DegreesX + (Degress(X)/Sec * Sec) (i.e. Integrate the Angular Velocity     
    GyroAngleY += GyroY * dT;
    GyroAngleZ += GyroZ * dT;
    
    //  Complementary Filter (for Pitch & Roll)
    bodyX = GyroAngleX * .96 + AccAngleX * .4;
    bodyY = GyroAngleY * .96 + AccAngleY * .4;
    bodyZ = GyroAngleZ;
    /*
    Serial.print("Pitch: ");
    Serial.print(bodyX);
    Serial.print(" // Yaw: ");
    Serial.print(bodyZ);
    Serial.print(" // Roll: ");
    Serial.print(bodyY);
    Serial.println();*/

  
}

//-------------------------------------------------------------------
//
//  //Calculate the X, Y, Z angles relative to the Inertial Frame 
//    the values of our Euler Angles)
//
//-------------------------------------------------------------------

void calculateEulerAngles(){

    //Convert All Measure Readouts back to Radians:
    bodyX *= PI/180.0;
    bodyY *= PI/180.0;
    bodyZ *= PI/180.0;

    GyroX *= PI/180.0;
    GyroY *= PI/180.0;
    GyroZ *= PI/180.0;

    //Calculate the Euler Angle Rates from Gyro Readouts
    velInertX = GyroX + GyroY*sin(inertialX)*tan(inertialY) + GyroZ*cos(inertialX)*tan(inertialY);
    velInertY = GyroY*cos(inertialX) - GyroZ*sin(inertialX);
    velInertZ = GyroY*(sin(inertialX)/cos(inertialY)) + GyroZ*(cos(inertialX)/cos(inertialY));

    //Integrate Euler Angle rates for final Euler Angle readout
    inertialX += (velInertX * dT); //Remove Difference in Time (dT), convert to Degrees for Servo use
    inertialY += (velInertY * dT); 
    inertialZ += (velInertZ * dT);    
    
    //Convert to Degrees
    panAngle = inertialZ * 180.0/PI;
    tiltAngle = inertialX * 180.0/PI;

    Serial.print("Tilt: ");
    Serial.print(tiltAngle);
    Serial.print("\tPan: ");
    Serial.print(panAngle);
    Serial.println();
}
