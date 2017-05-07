#include <Servo.h>



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
int pi = 3.14159265359;
Servo myServo, CarMotor;  //Steering Servo

// the setup routine runs once when you press reset:
void setup()
{
  pinMode(ChannelA , INPUT);
  pinMode(ChannelB , INPUT);
  myServo.attach(3);      //Attach Steering Servo to PWM Pin 2
  CarMotor.attach(5);
  inputString.reserve(50000);
  Serial.begin(115200);
}


//Main Program
void loop(){
  
      
  if(Serial.available() > 0){  
    char input = Serial.read();  

        carControl(input);
  
  }
  


  
}
void carControl(int input){

  switch(input){
    case 'a':
      myServo.write(65);
      //CarMotor.writeMicroseconds(1500);
      break;
    case 'b':
      myServo.write(70);
      //  CarMotor.writeMicroseconds(1500);
      break;
    case 'c':
      myServo.write(75);
      //CarMotor.writeMicroseconds(1500);
      break;
      case 'd':
      myServo.write(80);
      //CarMotor.writeMicroseconds(1500);
      break;
    case 'e':
      myServo.write(85);
      //  CarMotor.writeMicroseconds(1500);
      break;
    case 'z':
      myServo.write(90);
      //  CarMotor.writeMicroseconds(1500);
      break;
    case 'f':
      myServo.write(90);
      //CarMotor.writeMicroseconds(1500);
      break;
      case 'g':
      myServo.write(95);
      //  CarMotor.writeMicroseconds(1500);
      break;
    case 'h':
      myServo.write(100);
      //CarMotor.writeMicroseconds(1500);
      break;
      case 'i':
      myServo.write(105);
      //CarMotor.writeMicroseconds(1500);
      break;
    case 'j':
      myServo.write(110);
      //  CarMotor.writeMicroseconds(1500);
      break;
    case 'k':
      myServo.write(115);
      //CarMotor.writeMicroseconds(1500);
      break;
    default:
      myServo.write(90);    
  }

}

