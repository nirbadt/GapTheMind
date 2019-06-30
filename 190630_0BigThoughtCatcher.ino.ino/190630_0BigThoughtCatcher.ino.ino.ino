
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#define LED_PIN    6
#define LED_COUNT 24
#define length 1325 //400 for flower 1, 180 for flower 2, 1350 for floer 3
AccelStepper Stepper(AccelStepper::DRIVER, 5, 8); //use pin 12 and 13 for dir and step, 1 is the "external driver" mode (A4988)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//
// Pin number to change read or write mode on the shield
//
const int homePin = 2;     // the number of the pushbutton pin
const int enA = 9;
const int ledPin =  13;      // the number of the LED pin
int homeState = 0;         // variable for reading the pushbutton status

// Configure a DMX master controller, the master controller
// will use the RXEN_PIN to control its write operation 
// on the bus
 int i=0;
 int j=1;
 int homeAcc = 100;
 int homeSpeed = 100;
 int movingSpeed = 35;
 int movingAcc = 10;
 int homeGap = 0;
 int GAP = 12000;
 int dir = 1; //used to switch direction


void setup() {
  
  
  //Serial.begin(9600);
    // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(homePin, INPUT);
  pinMode(enA, OUTPUT);
  // Set channel 1 - 50 @ 50%
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)
  Serial.begin (9600);
  Stepper.move(-2000);
  Stepper.setAcceleration(homeAcc); //set acceleration (steps/second^2)
  Stepper.setMaxSpeed(homeSpeed); //set max speed the motor will turn (steps/second)
  while (homeState == LOW){
    homeState = digitalRead(homePin);
    //Serial.print(homeState);
    //Serial.println("\t" "going home");
    Serial.println ("going home");
    digitalWrite(ledPin, LOW);
    Stepper.run();
  }
  Serial.println("ET came home");
  digitalWrite(ledPin, HIGH);
  //Serial.print(Stepper.distanceToGo());
  //Stepper.move(0);
  Stepper.setCurrentPosition(0); 
  Stepper.setMaxSpeed(movingSpeed);
  Stepper.setAcceleration(movingAcc);
  dir = 1;
  delay(homeGap);

}
void loop() {
  int position;
  
  static int dimmer_val;
        //  Set pixel's color (in RAM)
  if(Stepper.distanceToGo()==0){ //check if motor has already finished his last move
    homeState = digitalRead(homePin);
       Serial.print(dir);
      Serial.print("\t");
      Serial.println(homeState);
    if (dir == 1 && homeState==LOW){
      Serial.print("Going home");
      Stepper.move(-1000);
      while(homeState==LOW){
      homeState = digitalRead(homePin);
      Serial.print("Searching for home");
      Serial.print(dir);
      Serial.print("\t");
      Serial.println(homeState);
      Stepper.run();  
      }
      Stepper.setCurrentPosition(0); 
      Serial.println("ET is back home");    
    }
    Stepper.move(length*dir); //set next movement to 1600 steps (if dir is -1 it will move -1600 -> opposite direction)
    dir = dir*(-1); //negate dir to make the next movement go in opposite direction
    digitalWrite(enA, HIGH);
    delay(random(GAP)); //Gap the mind
    digitalWrite(enA, LOW);
    //Serial.print(Stepper.distanceToGo());
    //Serial.println("End");
  }

  Serial.print ("distanceToGo");
  Serial.print ("\t");
  Serial.print (Stepper.distanceToGo());
  Serial.print ("position");
  Serial.print ("\t");
  Serial.println(position);
  Stepper.run(); //run the stepper. this has to be done over and over again to continously move the stepper
  if (dir == -1) { 
 position =  map(Stepper.distanceToGo(), 0, length, 255, 0);
  }
  if (dir == 1) { 
 position =  -map(Stepper.distanceToGo(), 0, length, 0, 255);
  }
//Serial.println(position);
     for(int i=0; i<strip.numPixels(); i++) { 
        strip.setPixelColor(i, position, position/4, 0); 
   }
   strip.show();                          //  Update strip to match
}
