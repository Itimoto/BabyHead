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

float orientX, tmpOrientX = 1;  //  OrientXYZ stores the XYZ values of the Body X Axis in the Inertial Ref Frame (ie relative to the Ground)
float orientY, tmpOrientY = 0;  //  When we rotate our IMU, we also rotate the X Axis and its corresponding components
float orientZ, tmpOrientZ = 0;  //  We can use the rotated components to find its polar coordinates (ie the Pan & Tilt values)

float panAngle, tiltAngle;
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
    AccAngleX = atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180/3.14159;
    AccAngleY = atan2(AccX, sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180/3.14159;
    
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

    //  Calculate the X, Y, Z values/coordinates of our new X Axis Vector
    /* The Actual Formula calculating the X, Y, Z components of a rotated Axis Vector x(b) from an original x(0) is
        x(b) = R(Z Axis Degrees Rotated)*R(Y Axis Degrees Rotated)*R(X Axis Degrees Rotated)*x(0)
        Where R(angle) is the Rotation Matrix
        Expanded, we find:
                [x(b)]   [                     cos(Y)cos(X) * x(0)  +                      cos(Y)sin(X) * y(0)  +       -sin(Y) * z(0)]
        x(b) =  [y(b)] = [ -cos(Z)sin(X)+sin(Z)sin(Y)cos(X) * x(0)  +   cos(Z)cos(X)+sin(Z)sin(Y)sin(X) * y(0)  +  sin(Z)cos(Y) * z(0)]
                [z(b)]   [  sin(Z)sin(x)+cos(Z)sin(Y)cos(X) * x(0)  +  -sin(Z)cos(X)+cos(Z)sin(Y)sin(X) * y(0)  +  cos(Z)cos(Y) * z(0)]
    
    */

    //Back to Radians:
    bodyX = bodyX * 3.14159/180.0;
    bodyY = bodyY * 3.14159/180.0;
    bodyZ = bodyZ * 3.14159/180.0;

    /*
    orientX = cos(bodyY)*cos(bodyX)*tmpOrientX + cos(bodyY)*sin(bodyX)*tmpOrientY - sin(bodyY)*tmpOrientZ;
    orientY = (-cos(bodyZ)*sin(bodyX)+sin(bodyZ)*sin(bodyY)*cos(bodyX))*tmpOrientX + (cos(bodyZ)*cos(bodyX)+sin(bodyZ)*sin(bodyY)*sin(bodyX))*tmpOrientY + sin(bodyZ)*cos(bodyY)*tmpOrientZ; //
    orientZ = (sin(bodyZ)*sin(bodyX)+cos(bodyZ)*sin(bodyY)*cos(bodyX))*tmpOrientX + (-sin(bodyZ)*cos(bodyX)+cos(bodyZ)*sin(bodyY)*sin(bodyX))*tmpOrientY + cos(bodyZ)*cos(bodyY)*tmpOrientZ;
    
    tmpOrientX = orientX; //Reset tmp values
    tmpOrientY = orientY;
    tmpOrientZ = orientZ;
    */

    

    print("X: ");
    print(orientX);
    print("\t Y: ");
    print(orientY);
    print("\t Z: ");
    print(orientZ);
    println();

    /*
    //  Convert to Polar Coordinates (to translate to Pan/Tilt Servo Motor commands)
    if(tmpOrientX == (0+lockTolerance) && tmpOrientY == (0+lockTolerance)){
      panAngle = bodyX; //Use X Axis Rotation for Pan instead of X/Y angle ratio, since 0/0 is undefined for trig functions (i.e. Gimbal Lock)
    } else {
      panAngle = atan2(orientY, orientX) * 180.0/3.14159;
    }
    tiltAngle = atan2(orientZ, sqrt(pow(orientX, 2) + pow(orientY, 2))) * 180/3.14159;
    
    Serial.print("Pan: ");
    Serial.print(panAngle);
    Serial.print("\t// Tilt: ");
    Serial.print(tiltAngle);
    Serial.println();
    Serial.println();*/
}
