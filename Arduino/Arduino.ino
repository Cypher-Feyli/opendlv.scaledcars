//Included Libraries
#include <Servo.h>
#include <Wire.h>
#include <Smartcar.h>
#include "RunningMedian.h"
#include <Adafruit_NeoPixel.h>
 
#define PIN 10
#define N_LEDS 16
 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);

RunningMedian IRB = RunningMedian(5);
RunningMedian IRBR = RunningMedian(5);
RunningMedian IRFR = RunningMedian(5);
RunningMedian USFR = RunningMedian(5);
RunningMedian USFC = RunningMedian(5);

String FrontRightIR = "";
String BackIR = "";
String BackRightIR = "";
String usFront = "";
String usFrontRight = "";

long counter = 0;
int reading = 0;
String traveledDistance = "";

const unsigned short GAIN =  0x09; //maximum gain
const unsigned short RANGE =  0x8C; //7 for 34 centimeters
SR04 front;
SR04 frontRight;
GP2D120 FrontRight;
GP2D120 BackRight;
GP2D120 Back;
// Define Variables:
const int ChannelA = 6;
const int ChannelB = 9;
bool delay1 = true;
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
int v = 0;
Odometer encoder;
const int encoderPin = 2;
int posofend;

int pi = 3.14159265359;
Servo myServo, CarMotor;  //Steering Servo

//http://playground.arduino.cc/Main/RotaryEncoders
//Odometer guides
//https://github.com/platisd/smartcar_shield/wiki/API-documentation#void-beginodometer-encoder
//
// the setup routine runs once when you press reset:
void setup()
{
  pinMode(ChannelA , INPUT);
  pinMode(ChannelB , INPUT);
  FrontRight.attach(A5);
  BackRight.attach(A4);
  Back.attach(A3);
  
  //front.attach(12,13);
  /*front.setGain(0);
  front.setRange(23);
  front.setPingDelay(200); 

  frontRight.setGain(GAIN);
  frontRight.setRange(RANGE);
  frontRight.setPingDelay(70); //uncomment if u want to use custom measurement range
  */
  
  myServo.attach(3);      //Attach Steering Servo to PWM Pin 2
  CarMotor.attach(5);
  encoder.attach(2);
  //encoder.attach(2, 4, HIGH); Commented out because of we dont need the output B from the wheel encoder.

  Serial.begin(115200);
  inputString.reserve(20000);
//  frontRight.attach(113);
  strip.begin();
  //While the handshake is running the middle LEDs (3,4) turns blue.
  handshakeLights(strip.Color(0,0,255));
  Wire.begin();                // join i2c bus (address optional for master)

  while (!Serial) {
   ; // wait for serial port to connect. Needed for native USB port only
  }
  establishContact();
  encoder.begin(); // begin measurement HERE

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
  posofend = inputString.indexOf("]");
  inputString.substring(posofstart+1,posofstart+2).toCharArray(dir, 2);
  angle = SetAngle(posofstart, posofend);
  if(angle > 20){

  //Two statements to sort out incorrect angles which cannot be executed if not changed.
  if(angle < 55){
    angle = 55;
  }
  else if( angle > 140){
    angle = 140;
  }

  //Depending on the angle of the steering and the direction that the car is moving the NeoPixels will be drawn with different colors
  //to indicate what driving commands the car is executing.
  if(angle > 60 && angle < 101){
    TurnOffBlinkers(0);
  }
  else if(angle < 61){
    drawLeftBlinker(strip.Color(255,150,0));
  }
  else if(angle >100){
    drawRightBlinker(strip.Color(255,150,0));
  }
  if(dir[0] == 'F' || dir[0] == 'K'){
    directionLights(strip.Color(0,255,0));
  }
  else if(dir[0] == 'B'){
    directionLights(strip.Color(255,0,0));
  }
  else if(dir[0] == 'S'){
    directionLights(strip.Color(32,32,32));
  }
    switch (dir[0]) {
      case 'F': //rotate counter-clockwise going forward
        CarMotor.writeMicroseconds(1552);
        myServo.write(angle);
        break;
      case 'K': //rotate counter-clockwise going forward
        CarMotor.writeMicroseconds(1553);
        myServo.write(angle);
        break;
      case 'S'://stop car
        myServo.write(angle);
        CarMotor.writeMicroseconds(1500);
        break;
      case 'B': //turn clock-wise
        CarMotor.writeMicroseconds(1245);
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
         default: 
         myServo.write(90);
         CarMotor.writeMicroseconds(1500);
         break;
         
  }
  }
}
//Handshake which only sends the string [H] until proper connection has been established with the proxy.
void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("[H]");   // send an initial string
    delay(500);
  }
}

int SetAngle(int posofstart, int posofend){
  v = (inputString.substring(posofstart+2,posofend).toInt()); 
  return v;
}
void serialEvent() {
  while (Serial.available() && !stringComplete) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if(inChar == '['){
      breakLoop = true;
    }
    if(breakLoop){
      inputString += inChar;
    }
    if (inChar == ']' && breakLoop) {
      stringComplete = true;
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
  IRFR.add(float(FrontRight.getDistance()));
  IRBR.add(float(Back.getDistance()));
  IRB.add(float(BackRight.getDistance()));
  //USFR.add(frontRight.getDistance());
  USFC.add(float(ReadSensor()));
  traveledDistance = String((encoder.getDistance()*4.2)); 
  Serial.println("[V." + traveledDistance + "]");
  counter++;
  if(counter == 5){
  
  FrontRightIR = String(IRFR.getMedian());
  BackIR = String(IRBR.getMedian());
  BackRightIR = String(IRB.getMedian());
  //usFrontRight = String(USFR.getMedian());
  usFront = String(USFC.getMedian());
  IRFR.clear();
  IRBR.clear();
  IRB.clear();
  USFC.clear();
  //USFR.clear();
  //get distance traveled since begin() in setup()

  //Serial.println("[IR.22,44;66]"); This is for testing purposes

  Serial.println("[IR." + FrontRightIR + "," + BackIR + ";" + BackRightIR +"]");
  //Serial.println("[US.100]"); 
  // Printing ultrasonic as 100 every time because we dont have functioning ultrasonic at the moment.
  Serial.println("[US." + usFront + "]");
  counter = 0;

 }
}

void setGain(unsigned short gainValue){
    Wire.beginTransmission(0x70); //start i2c transmission
    Wire.write(0x01); //write to GAIN register (1)
    Wire.write(constrain(gainValue,0,31)); //write the value
    Wire.endTransmission(); //end transmission
}

void setRange(unsigned short rangeValue){
    Wire.beginTransmission(0x70); //start i2c transmission
    Wire.write(0x02); //write to range register (1)
    Wire.write(rangeValue); //write the value -> Max_Range = (rangeValue * 3.4) + 3.4 in centimeters
    Wire.endTransmission(); //end transmission
}

//https://www.arduino.cc/en/Tutorial/SFRRangerReader
int ReadSensor(){
  setGain(0);
  setRange(23);
   // step 1: instruct sensor to read echoes
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
  // use 0x51 for centimeters
  // use 0x52 for ping microseconds
  Wire.endTransmission();      // stop transmitting

  // step 2: wait for readings to happen
  delay(20);                   // datasheet suggests at least 65 milliseconds

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(112); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
    return reading;
}
//Functions for drawing the LEDs on the car to the color which is inputed when calling the functions.
//Draws the middle LEDs.
static void directionLights(uint32_t c) {
      strip.setPixelColor(3  , c); // Draw new pixel
      strip.setPixelColor(4  , c); // Draw new pixel
      strip.show();
}
//Draws the middle LEDs.
static void handshakeLights(uint32_t c) {
      strip.setPixelColor(0  , 0); // Draw new pixel
      strip.setPixelColor(1  , 0); // Draw new pixel
      strip.setPixelColor(3  , c); // Draw new pixel
      strip.setPixelColor(4  , c); // Draw new pixel
      strip.setPixelColor(6  , 0); // Draw new pixel
      strip.setPixelColor(7  , 0); // Draw new pixel
      strip.show();
}
//Draws the left blinkers.
static void drawLeftBlinker(uint32_t c) {
      strip.setPixelColor(6  , 0); // Draw new pixel
      strip.setPixelColor(7  , 0); // Draw new pixel
      strip.setPixelColor(0  , c); // Draw new pixel
      strip.setPixelColor(1  , c); // Draw new pixel
      strip.show();
}
//Draws the right blinkers.
static void drawRightBlinker(uint32_t c) {
      strip.setPixelColor(6  , c); // Draw new pixel
      strip.setPixelColor(7  , c); // Draw new pixel
      strip.setPixelColor(0  , 0); // Draw new pixel
      strip.setPixelColor(1  , 0); // Draw new pixel
      strip.show();
}
//Draws all the blinkers pixels
static void TurnOffBlinkers(uint32_t c) {
      strip.setPixelColor(0  , 0); // Draw new pixel
      strip.setPixelColor(1  , 0); // Draw new pixel
      strip.setPixelColor(3  , c); // Draw new pixel
      strip.setPixelColor(4  , c); // Draw new pixel
      strip.setPixelColor(6  , 0); // Draw new pixel
      strip.setPixelColor(7  , 0); // Draw new pixel
      strip.show();
}
