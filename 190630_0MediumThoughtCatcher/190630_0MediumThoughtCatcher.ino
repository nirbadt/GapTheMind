
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#define LED_PIN    6
#define LED_COUNT 16
#define length 380 //380 for flower 1, 180 for flower 2, 1350 for floer 3
AccelStepper Stepper(AccelStepper::DRIVER, 5, 8); //use pin 12 and 13 for dir and step, 1 is the "external driver" mode (A4988)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//
// Pin number to change read or write mode on the shield
//
const int homePin = 2;     // the number of the pushbutton pin
const int sitPin = 3;     // the number of the pushbutton pin

const int enA = 9;
const int ledPin =  13;      // the number of the LED pin
int homeState = 0;         // variable for reading the pushbutton status
int sitState = 0;         // variable for reading the pushbutton status
int lasthomeState = HIGH;         // variable for reading the pushbutton status
int lastsitState = LOW;         // variable for reading the pushbutton status
 int i=0;
 int j=1;
 int homeAcc = 500;
 int homeSpeed = 500;
 int movingSpeed = 25;
 int movingAcc = 5;
 int sittingSpeed = 25;
 int sittingAcc = 5;
 int isExpanding = 1;
 int homeGap = 0;
 int GAP = 2000;
 int upperStandbyGap = 1000;
 int lowerStandbyGap = 1000;
 int uppersittingGap = 1000;
 int lowersittingGap = 1000;
 int dir = 1; //used to switch direction
 int steps = 3; //2 steps is minimum with current implementation - fix this in the future
 int breath = 0;
 int expand = length/steps;
 int counter = 1;
 int shortCoef = 1;
 int longCoef = 2;
 float portion;

void setup() {
  
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(homePin, INPUT);
  pinMode(sitPin, INPUT);
  pinMode(enA, OUTPUT);
  // Set channel 1 - 50 @ 50%
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about max
  Serial.begin (9600);
  Stepper.move(-2000); //send motor home
  Stepper.setAcceleration(homeAcc); //set acceleration (steps/second^2)
  Stepper.setMaxSpeed(homeSpeed); //set max speed the motor will turn (steps/second)
  Serial.println ("Searching home");
  while (homeState == LOW){ 
    homeState = digitalRead(homePin);
    //Serial.print(homeState);
    //Serial.println("\t" "going home");
    digitalWrite(ledPin, LOW);
    Stepper.run();
  }
  Serial.println("Found home");
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
  int LED_Light;
  lasthomeState = homeState;
  lastsitState =sitState;
  sitState = digitalRead(sitPin);
  homeState = digitalRead(homePin);
  if (sitState ==LOW){
    //Serial.println ("Goodbye friend");
    friend_left();
    //Serial.println ("Friend left");
    //delay(GAP);
    Stepper.move(length/2);
    Serial.println("Standby mode");
    while (sitState == LOW) { //standby mode
    lastsitState =sitState;
    sitState = digitalRead(sitPin);
    while (Stepper.distanceToGo()!=0){
    Stepper.run();
    }
    digitalWrite(enA, HIGH);
    }
    digitalWrite(enA, LOW);
  } 
  if (sitState == HIGH && lastsitState == LOW){ //a person has sat on the chair
    Serial.println("Hello friend");
    friend_came();
    //Serial.println ("Friend came");
    //delay(GAP);
    while (Stepper.distanceToGo()!=0){
    Stepper.run();
    }
  }

  /*Serial.print ("homeState");
  Serial.print ("\t");
  Serial.print (homeState);
  Serial.print ("\t");
  Serial.print ("lasthomeState");
  Serial.print ("\t");
  Serial.println (lasthomeState);*/
  if (Stepper.distanceToGo()!=0 && homeState == HIGH && lasthomeState == LOW){ //check if motor arrived home beofore finishing its move
      //Serial.println("entered home early state");
      Stepper.setCurrentPosition(0); 
      Serial.println("I came home early");
      lasthomeState = homeState;
      dir = 1;
      delay(GAP);  
  }
  if(Stepper.distanceToGo()==0){ //check if motor has already finished his last move
    homeState = digitalRead(homePin);
    if (dir == 1 && homeState==LOW){ //check if motor finished its down move but didn't reach the home sensor
      Serial.println("Searching home");
      Stepper.move(-1000);
      while(homeState==LOW){
      homeState = digitalRead(homePin);
      Stepper.run();  
      delay(20);
      }
      Serial.println("Found home");
      Stepper.setCurrentPosition(0); 
      dir = 1; 
     }
    else if (dir == 1 && homeState==HIGH) { //check if motor finished its move and did reach the home sensor
      Serial.println ("Got home safe");
      dir = 1;  
    }
    if(homeState==HIGH && Stepper.distanceToGo()==0 && isExpanding == true){ // when reaching home and is expanding, increase counter 
      counter ++;
      //Serial.println("counter increased");
      Serial.println("Long Gap");    
      delay(GAP*counter*2); //Gap the mind
    }
    if(homeState==HIGH && Stepper.distanceToGo()==0 && isExpanding == false){ // when reaching home and is retracting, decrease counter   
      counter --;
      //Serial.println("counter decreased");
      Serial.println("Long Gap");
      digitalWrite(enA, HIGH);    
      delay(GAP*counter*longCoef); //Gap the mind
      digitalWrite(enA, LOW);
    }
    if(homeState==LOW && Stepper.distanceToGo()==0){ //when fully open make a short gap
      Serial.println("Short Gap");
      digitalWrite(enA, HIGH);
      delay(GAP*counter*shortCoef); //Gap the mind
      digitalWrite(enA, LOW);
    }
    if (counter == steps) {
      isExpanding = 0;
      Serial.println("retracting");
    }
    if (counter == 1) {
      isExpanding = 1;
      Serial.println("expanding");
    }   
    breath = (counter*expand);  
    portion = 255*((float) breath)/ ((float) length);
    Stepper.move(breath*dir); //set next movement to 1600 steps (if dir is -1 it will move -1600 -> opposite direction)
    Serial.print("breath   ");
    Serial.print(breath);
    Serial.print("   portion   ");
    Serial.print(portion);
    Serial.print("   Stepper.distanceToGo   ");
    Serial.println(Stepper.distanceToGo());
    dir = dir*(-1); //negate dir to make the next movement go in opposite direction
    //Serial.print(Stepper.distanceToGo());
    //Serial.println("End");
  }

  Stepper.run(); //run the stepper. this has to be done over and over again to continously move the stepper
  if (dir == -1) { 
    
    LED_Light =  map(Stepper.distanceToGo(), 0, breath, portion, 0);
 //LED_Light  = constrain(LED_Light, 0, 255);
  }
  if (dir == 1) { 
    LED_Light =  -map(Stepper.distanceToGo(), 0, breath, 0, portion);
 //LED_Light  = constrain(LED_Light, 0, 255);
  }
Serial.print("breath   ");   
Serial.print(breath);   
Serial.print("   Stepper.distanceToGo   ");   
Serial.print(Stepper.distanceToGo());  
Serial.print("   LED_Light   ");  
Serial.println(LED_Light);
     for(int i=0; i<strip.numPixels(); i++) { 
        strip.setPixelColor(i, LED_Light, LED_Light/4, 0); 
   }
   strip.show();                          //  Update strip to match
}

void friend_came()
{
  homeState = digitalRead(homePin);
  //Serial.println("entered friend_came");
  Stepper.move(-2000);
  Stepper.setAcceleration(homeAcc); //set acceleration (steps/second^2)
  Stepper.setMaxSpeed(homeSpeed); //set max speed the motor will turn (steps/second)
  while (homeState == LOW){
    homeState = digitalRead(homePin);
    Stepper.move(-2000);
    digitalWrite(ledPin, LOW);
    Stepper.run();
    Serial.println(Stepper.distanceToGo());
  }
  //Serial.println("Hiding at home");
  digitalWrite(ledPin, HIGH);
  //Serial.print(Stepper.distanceToGo());
  //Stepper.move(0);
  Stepper.setCurrentPosition(0); 
  Stepper.setMaxSpeed(movingSpeed);
  Stepper.setAcceleration(movingAcc);
  dir = 1;
  counter = 0;
  //Serial.println("left friend_came");
  //delay(3000);
}


void friend_left()
{
  //Serial.println("entered friend_left");
  Stepper.move(-2000);
  Stepper.setAcceleration(homeAcc); //set acceleration (steps/second^2)
  Stepper.setMaxSpeed(homeSpeed); //set max speed the motor will turn (steps/second)
  while (homeState == LOW){
    homeState = digitalRead(homePin);
    Stepper.move(-2000);
    digitalWrite(ledPin, LOW);
    Stepper.run();
    Serial.println(Stepper.distanceToGo());
  }
  //Serial.println("Hiding at home");
  digitalWrite(ledPin, HIGH);
  //Serial.print(Stepper.distanceToGo());
  //Stepper.move(0);
  Stepper.setCurrentPosition(0); 
  Stepper.setMaxSpeed(movingSpeed);
  Stepper.setAcceleration(movingAcc);
  dir = 1;
  counter = 0;
  //Serial.println("leaving friend_left");
  //delay(3000);
}
