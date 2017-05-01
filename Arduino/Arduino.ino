String incomingByte = "";   // for incoming serial data

int inData[10];  // Allocate some space for the Bytes
byte inByte;   // Where to store the Bytes read

//Included Libraries
#include <Servo.h>
#include <Wire.h>
#include <Smartcar.h>
#include "RunningMedian.h"

RunningMedian IRB = RunningMedian(5);
RunningMedian IRBR = RunningMedian(5);
RunningMedian IRFR = RunningMedian(5);
RunningMedian USFR = RunningMedian(5);
RunningMedian USFC = RunningMedian(5);

long counter = 0;

const unsigned short GAIN = 0; //maximum gain
const unsigned short RANGE = 23; //7 for 34 centimeters
SRF08 front;
SRF08 frontRight;
GP2D120 FrontRight;
GP2D120 BackRight;
GP2D120 Back;
// Define Variables:
const int ChannelA = 6;
const int ChannelB = 9;

//Signal Conditioning limits
const int lo = 800;
const int hi = 2000;
const int deadlo = 1470;
const int deadhi = 1530;
const int center = 1500;
boolean stringComplete = false;
boolean a = true;
boolean b = true;
boolean breakLoop = false;
String inputString = "";
char dir[1];
int angle = 0;
int posofstart = 0;
int Degrees;
int Speed;
const int numReadings = 7;
int currRead = 0;

int pi = 3.14159265359;
Servo myServo, CarMotor;  //Steering Servo

// the setup routine runs once when you press reset:
void setup()
{
  pinMode(ChannelA , INPUT);
  pinMode(ChannelB , INPUT);
  FrontRight.attach(A5);
  BackRight.attach(A4);
  Back.attach(A3);
  
  front.attach(112);
  //front.setGain(GAIN);
  //front.setRange(RANGE);
  //front.setPingDelay(70); 
  /*frontRight.attach(113);
  frontRight.setGain(GAIN);
  frontRight.setRange(RANGE);
  frontRight.setPingDelay(70); //uncomment if u want to use custom measurement range
*/
  myServo.attach(3);      //Attach Steering Servo to PWM Pin 2
  CarMotor.attach(5);
  inputString.reserve(25000);
  Serial.begin(115200);

}
//Main Program
void loop()
{
  int a = pulseIn(ChannelB, HIGH, 35000);
  if (a < 700) {
    a = true;
    if(stringComplete){
    handleInput();
    inputString = "";
    stringComplete = false;
    }
    NormalizeSensValues();
  }
  else {
    b = true;
    SteerCar();
  }
}

void handleInput() { //handle serial input if there is any
  if (b) {
    CarMotor.writeMicroseconds(1500);
    myServo.write(90);
    b = false;
  }
  posofstart = inputString.indexOf("[");
  inputString.substring(posofstart+1,posofstart+2).toCharArray(dir, 2);
  angle = SetAngle(posofstart);
  //Serial.println(angle);
  if(angle < 60){
    angle = 60;
  }
  else if( angle > 120){
    angle = 120;
  }
    switch (dir[0]) {
      case 'F': //rotate counter-clockwise going forward
        CarMotor.writeMicroseconds(1554);
        myServo.write(angle);
        break;
      case 'S'://stop car
        myServo.write(angle);
        CarMotor.writeMicroseconds(1500);
        break;
      case 'B': //turn clock-wise
        CarMotor.writeMicroseconds(1290);
        myServo.write(angle);
        break;
      /* case 'r': //turn clock-wise
         CarMotor.writeMicroseconds(1560);
         myServo.write(110);
         break;
        case 'f': //Forward specific
         CarMotor.writeMicroseconds(1560);
         myServo.write(90);
         break;
        case 'c': //Change speed
         CarMotor.writeMicroseconds(1560);
         break;
        case 'o': //Turn left
         myServo.write(55);
         break;
        case 'p': //Turn right
         myServo.write(110);
         break;
        case 'B': //go back
         CarMotor.writeMicroseconds(1290);
         myServo.write(90);
         break;
        case 'S'://stop car
         myServo.write(90);
         CarMotor.writeMicroseconds(1500);
         break;*/
  }

}
int SetAngle(int posofstart){
  int posofend = inputString.indexOf("]");
  int v = 0;
  v = (inputString.substring(posofstart+2,posofend).toInt()); 
  return v;
}
void serialEvent() {
  while (Serial.available() && !stringComplete) {
    // get the new byte:
    char inChar = (char)Serial.read();
    inputString += inChar;
    if(inChar == '['){
      breakLoop = true;
    }
    if (inChar == ']' && breakLoop) {
      stringComplete = true;
      break;
    }
  }
}

void SteerCar() {
  int ch[2];
  if (a) {
    CarMotor.writeMicroseconds(1500);
    myServo.write(90);
    a = false;
  }
  ch[1] = pulseIn(ChannelA, HIGH);
  ch[0] = pulseIn (ChannelB, HIGH);//Read and store channel 1
  for (int i = 0; i <= 1; i++)  //Signal Conditioning loop
  {
    if (ch[i] <= lo)             //Trim Noise from bottom end
    {
      ch[i] = 1500;
    }

    if (ch[i] <= deadhi && ch[i] >= deadlo)     //Create Dead-Band
    {
      ch[i] = center;
    }

    if (ch[i] >= hi)            //Trim Noise from top end
    {
      ch[i] = hi;
    }
  }

  //Steering Control Output on Channel 4
  Degrees = ch[0];
  if (Degrees >= lo && Degrees <= deadlo)
  {
    Degrees = map(Degrees, lo, deadlo, 0, 90);
  }
  else if (Degrees >= deadlo && Degrees <= deadhi)
  {
    Degrees = 90;
  }
  else if (Degrees >= deadhi && Degrees <= hi)
  {
    Degrees = map(Degrees, deadhi, hi, 90, 180);
  }

  Speed = ch[1];
  if (Speed >= lo && Speed <= deadlo)
  {
    Speed = map(Speed, lo, deadlo, 1150, 1400);
  }
  else if (Speed >= deadlo && Speed <= deadhi)
  {
    Speed = 1500;
  }
  else if (Speed >= deadhi && Speed <= hi)
  {
    Speed = map(Speed, deadhi, hi, 1540, 1570);
  }
  myServo.write(Degrees);
  CarMotor.writeMicroseconds(Speed);

}


void NormalizeSensValues() {
  //school testing
  Serial.println("[IR.22,44;66]");
  Serial.println("[US.33]");
  /*delay(100);
  IRFR.add(FrontRight.getDistance());
  IRBR.add(Back.getDistance());
  IRB.add(BackRight.getDistance());
  //USFR.add(frontRight.getDistance());
  USFC.add(front.getDistance());
  counter++;
  if(counter == 5){
  counter = 0;
  String FrontRightIR = String(int(IRFR.getMedian()));
  String BackIR = String(int(IRBR.getMedian()));
  String BackRightIR = String(int(IRB.getMedian()));
  //String usFrontRight = String(int(USFR.getMedian()));
  String usFront = String(int(USFC.getMedian()));
  Serial.println("[IR" + FrontRightIR + "," + BackIR + ";" + BackRightIR +"]");
  Serial.println("[US" + usFront + "]");
  }*/
}
