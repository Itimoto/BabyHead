/*----------------------------------------------------------------------------------
 * 1.19.2020 ~ 10:33PM ~ Dimitry Melnikov
 * 
 * babyHead_Camera:
 *  Reads the Pan/Tilt from babyHead_Goggles and moves the
 *  Camera to the indicated position, mimicking the head movement
 *  
 *----------------------------------------------------------------------------------  
*/

//  Is it really that bad to comment every line of code?
//    I feel like it usually is, but--in this context, where
//    nigh' everything is new, I feel like it's needed

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Servo.h>

int radioPinCE = 9;
int radioPinCSN = 8;
const byte panTiltChannel[6] = "00001";

RF24 radio(radioPinCE, radioPinCSN);

float recievedPanAngle = 90;
float recievedTiltAngle = 90;

int panServoPin = 5;
int tiltServoPin = 6;
int panPlusServoPin = 3;

Servo panServo;
Servo tiltServo;
Servo panPlusServo; //  Extends the range of our Servo (accurate +- 180 Degree Range)

void setup() {
    Serial.begin(9600);

    radio.begin();
    radio.openReadingPipe(0, panTiltChannel); //  Set address for Reading Pan/Tilt data from _Goggles
    radio.startListening();                   //  Set nRF as a Reciever

    panServo.attach(panServoPin);
    tiltServo.attach(tiltServoPin);
    panPlusServo.attach(panPlusServoPin);

    panPlusServo.write(90);
    
    Serial.println("Initialized");    
}

void loop() {
    if(radio.available()){
      byte payLoad[8] = {0};                  //  8 Byte Array for storing our recieved Pan/Tilt values
      radio.read(&payLoad, sizeof(payLoad));  //  Deposits Input Data into Payload   

      byte *rxPanComponent = (byte *) &recievedPanAngle;    //  Points towards the individual Bytes making up the recievedPanAngle Float
      byte *rxTiltComponent = (byte *) &recievedTiltAngle;  //    See babyHead_Goggles for details
      
      for(int i = 0; i < 4; i++){             //  Deposits payLoad data into the individual Bytes; See babyHead_Goggles for details
        rxPanComponent[i] = payLoad[i];       //  Equivalent to: *((byte *) &recievedPanAngle + i) = payLoad[i]; or 
                                              //  'The Value stored at' 'The Byte' 'Of the Variable Address / Stack Address of recievedPanAngle' 'incremented by i'
      }
      for(int i = 0; i < 4; i++){
        rxTiltComponent[i] = payLoad[i+4];
      }    
    }

    
    
    if(recievedPanAngle > 0 && recievedPanAngle < 180){ //Keeps the Servo within its bounds
      panServo.write(recievedPanAngle);
    } else if (recievedPanAngle > -90 && recievedPanAngle <= 0){
      panPlusServo.write(+1*(recievedPanAngle+90));    //  ::PanPlusServo initialized @ 90deg. When Pan<0, we'll move CW
    } else if (recievedPanAngle >= 180 && recievedPanAngle < 270) {
      panPlusServo.write(+1*(recievedPanAngle-90));    //  :: Moves us CW, along the 'RHS'
    }

    
    if(recievedTiltAngle > 0 && recievedTiltAngle < 180){
      tiltServo.write(recievedTiltAngle);
    }

    Serial.print("Pan: ");
    Serial.print(recievedPanAngle);
    Serial.print("Tilt: ");
    Serial.print(recievedTiltAngle);
    Serial.println();
}
