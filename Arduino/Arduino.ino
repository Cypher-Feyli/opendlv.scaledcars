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

const unsigned short GAIN =  0x09; //maximum gain
const unsigned short RANGE =  0x8C; //7 for 34 centimeters

GP2D120 FrontRight;
GP2D120 BackRight;
GP2D120 Back;

// Define Variables:
const int ChannelA = 6;
const int ChannelB = 9;
String FrontRightIR = "";
String BackIR = "";
String BackRightIR = "";
String usFront = "";
String usFrontRight = "";

long counter = 0;
int reading = 0;
String traveledDistance = "";

//Signal Conditioning limits
const int lo = 800;
const int hi = 2000;
const int deadlo = 1470;
const int deadhi = 1530;
const int center = 1500;
//used booleans in code
boolean stringComplete = false;
boolean a = true;
boolean b = true;
boolean breakLoop = false;
//variables
String inputString = "";
char dir[1];
int angle = 0;
int posofstart = 0;
int Degrees;
int Speed;
int v = 0;
Odometer encoder;
const int encoderPin = 2;
int posofend;

int pi = 3.14159265359;
Servo myServo, CarMotor;  //Steering Servo

//The setup routine runs once when you press reset button on the arduino board
void setup()
{
  pinMode(ChannelA , INPUT);
  pinMode(ChannelB , INPUT);
  FrontRight.attach(A5);
  BackRight.attach(A4);
  Back.attach(A3);

  myServo.attach(3);      //Attach Steering Servo to PWM Pin 2
  CarMotor.attach(5);
  encoder.attach(2);
  Serial.begin(115200);
  inputString.reserve(20000);
  //  frontRight.attach(113);
  strip.begin();
  //While the handshake is running the middle LEDs (3,4) turns blue.
  handshakeLights(strip.Color(0, 0, 255));
  Wire.begin();                // join i2c bus (address optional for master)

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  establishContact();
  encoder.begin(); // Begin measurements here
}
//Main Program
void loop()
{
  //Pulsing in the
  int pulse = pulseIn(ChannelB, HIGH, 35000);
  if (pulse < 700) {
    a = true;
    if (stringComplete) {
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

void handleInput() { //Handle serial input if there is any
  if (b) {
    CarMotor.writeMicroseconds(1500);
    myServo.write(81);
    b = false;
  }
  //The two delimiters
  posofstart = inputString.lastIndexOf("[");
  posofend = inputString.lastIndexOf("]");
  inputString.substring(posofstart + 1, posofstart + 2).toCharArray(dir, 2);
  angle = SetAngle(posofstart, posofend);
  //Due to an issue when getting only one value we filter it out
  //Did not happen often and did not affect in any major way but still required a corner case fix
  if (angle > 20) {
    //Two statements to sort out incorrect angles which cannot be executed if not changed.

    //If angle is too small
    if (angle < 55) {
      angle = 55;
    }
    //If angle is too big
    else if ( angle > 140) {
      angle = 140;
    }

    //Depending on the angle of the steering and the direction that the car is moving the NeoPixels will be drawn with different colors
    //to indicate what driving commands the car is executing.
    if (angle > 60 && angle < 101) {
      TurnOffBlinkers(0);
    }
    else if (angle < 61) {
      drawLeftBlinker(strip.Color(255, 150, 0));
    }
    else if (angle > 100) {
      drawRightBlinker(strip.Color(255, 150, 0));
    }
    if (dir[0] == 'F' || dir[0] == 'K') {
      directionLights(strip.Color(0, 255, 0));
    }
    else if (dir[0] == 'B') {
      directionLights(strip.Color(255, 0, 0));
    }
    else if (dir[0] == 'S') {
      directionLights(strip.Color(32, 32, 32));
    }
    //Switch statement with direction
    switch (dir[0]) {
      case 'F': //Forward
        CarMotor.writeMicroseconds(1554);
        myServo.write(angle);
        break;
      case 'K': //Forward slowly
        CarMotor.writeMicroseconds(1553);
        myServo.write(angle);
        break;
      case 'S'://Stop 
        myServo.write(angle);
        CarMotor.writeMicroseconds(1500);
        break;
      case 'B': //Backwards slowly
        CarMotor.writeMicroseconds(1245);
        myServo.write(angle);
        break;
      default: //Stop and set angle to straight forward
        myServo.write(81);
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
//extracts the angle from the string
int SetAngle(int posofstart, int posofend) {
  v = (inputString.substring(posofstart + 2, posofend).toInt());
  return v;
}
//SerialEvent from arduino built in library
void serialEvent() {
  while (Serial.available() && !stringComplete) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == '[') {
      breakLoop = true;
    }
    if (breakLoop) {
      inputString += inChar;
    }
    if (inChar == ']' && breakLoop) {
      stringComplete = true;
    }
  }
}
//Used to steer with the rc controller
void SteerCar() {
  int ch[2];
  if (a) {
    CarMotor.writeMicroseconds(1500);
    myServo.write(81);
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

  //Steering Control Output on Channel 0
  //mapping out from 0-180
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
  //mapping out values for speed
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
/**
   Function to normalize sensor values by getting the median values
   Using the counter to measure how many times we measured values and then
   using the runningMedian library to fetch the library
*/
void NormalizeSensValues() {
  IRFR.add(float(FrontRight.getDistance()));
  IRBR.add(float(Back.getDistance()));
  IRB.add(float(BackRight.getDistance()));
  USFC.add(float(ReadSensor()));
  traveledDistance = String((encoder.getDistance() * 4.2));
  Serial.println("[V." + traveledDistance + "]");//get distance traveled since begin() in setup()
  counter++;
  //when counter hits 5 we get the median.
  if (counter == 5) {
    FrontRightIR = String(IRFR.getMedian());
    BackIR = String(IRBR.getMedian());
    BackRightIR = String(IRB.getMedian());
    usFront = String(USFC.getMedian());
    //clear the medians
    IRFR.clear();
    IRBR.clear();
    IRB.clear();
    USFC.clear();
    //sending the us values
    Serial.println("[IR." + FrontRightIR + "," + BackIR + ";" + BackRightIR + "]");
    Serial.println("[US." + usFront + "]");
    //for testing Serial.println("[US.100]");

    counter = 0;

  }
}
/**
   Smartcar shield library
*/
void setGain(unsigned short gainValue) {
  Wire.beginTransmission(0x70); //start i2c transmission
  Wire.write(0x01); //write to GAIN register (1)
  Wire.write(constrain(gainValue, 0, 31)); //write the value
  Wire.endTransmission(); //end transmission
}
/**
   Smartcar shield library
*/
void setRange(unsigned short rangeValue) {
  Wire.beginTransmission(0x70); //start i2c transmission
  Wire.write(0x02); //write to range register (1)
  Wire.write(rangeValue); //write the value -> Max_Range = (rangeValue * 3.4) + 3.4 in centimeters
  Wire.endTransmission(); //end transmission
}
/**
   Adjusted SFR08RangerReader with our own implementations
   setGain
   setRange
   delay 20
   https://www.arduino.cc/en/Tutorial/SFRRangerReader
*/
int ReadSensor() {
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
  delay(20);                   // datasheet suggests at least 65 milliseconds lowered our to 20 for faster processing

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
