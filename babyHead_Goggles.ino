/*----------------------------------------------------------------------------------
 * 1.19.2020 ~ 10:33PM ~ Dimitry Melnikov
 * 
 * babyHead_Goggles:
 *  Reads MPU6050 (Gyro & Accel) Data, converts it to Euler Angles,
 *  and sends the 'Pan/Tilt' Information to babyHead_Camera to move 
 *  the Camera to the indicated position
 *  
 *----------------------------------------------------------------------------------
*/

//  Is it really that bad to comment every line of code?
//    I feel like it usually is, but--in this context, where
//    nigh' everything is new, I feel like it's needed

#include <Wire.h>     //The Wire Library allows an Arduino to communicate with I2C (like the MPU6050) and TWI Devices

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

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

int radioPinCE = 9;                       //  Chip-Enable Pin for the nRF module
int radioPinCSN = 8;                      //  Chip-Select-Not Pin for the nRF Module
const byte panTiltChannel[6] = "00001";   //  Radio Pipe Address through which _Gogg & _Cam communicate

RF24 radio(radioPinCE, radioPinCSN);


void setup() {
    Serial.begin(9600);
    
    Wire.begin();                   //  Start Communicatin'
    Wire.beginTransmission(MPU);    //  Open a channel with the MPU (which is at register 0x6B)
    Wire.write(0x6B);               //  Talk to Register 0x6B, which contains the DEVICE_RESET Bit
    Wire.write(0x00);               //  Reset the MPU by writing in 0 (sets DEVICE_RESET to 0)
    Wire.endTransmission(true);

    radio.begin();
    radio.openWritingPipe(panTiltChannel);  //  Set address for Writing output data
    radio.stopListening();                  //  Set nRF as a transmitter
    
    Serial.println("Initialized");    
}

void loop() {
    
    readMPU();                    //  Read necessary data from the MPU
    calculateEulerAngles();       //  Calculate the values needed to orient the Pan/Tilt Servo

    panAngle += 90;               //Bring the Pan/Tilt into a Servos working range
    tiltAngle += 90;


    byte payLoad[8];                            //  Will store both our Pan Angle & Tilt Angle
    byte *panComponent = (byte *) &panAngle;    //  Points to our panAngle's Address; Casting it to a byte pointer 
                                                //    lets us increment it by individual addresses/bytes, since 1 Byte 
                                                //    takes up 1 Byte (1 Address) and 1 Float takes up 4 Bytes 
                                                //    (4 Addresses) (and one increment on a Float pointer would mean 
                                                //    moving by 4 Addresses instead of 1, since we want the individual
                                                //    component Bytes (each individual address) to place in the byte Array)
    byte *tiltComponent = (byte *) &tiltAngle;  //  Ditto ^^^
    
    for(int i = 0; i < 4; i++){               //  We'll place the components of the panAngle Float variable into the payLoad, Byte-by-Byte
      payLoad[i] = panComponent[i];           //    Equivalent to: payLoad[i] = *(panComponent + i); or payLoad[i] = *((byte *) &panAngle + i);
    }                                         //    (i.e. We're looking at the individual Bytes making up our Float at its Address, then 
                                              //    'looking' further down while copying them into the payLoad
    for(int i = 0; i < 4; i++){                 //  Ditto ^^^
      payLoad[i+4] = tiltComponent[i];          //  (Shift over to account for panAngle components)  
    }
    
    radio.write(&payLoad, sizeof(payLoad));   //  Send the combined Pan, Tilt Values to babyHead_Camera
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
    GyroY = ((Wire.read() << 8 | Wire.read()) - 132.5)  / 131.0; //
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
