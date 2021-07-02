/*
  Code for controlling Tesla Door Handle
  motor R opens, Motor L closes, O turns motor off
   Pin definitions can be changed and adapted to whatever you like.
   Use Pullup resistors on limit switches
   Connect motor to 5-12v dc, both work just fine.
   I am using an MFRC522 RFID card reader to unlock the door but the code could be easily adapted to suit a host of purpuses :)
*/

//-----RFID Libraries and definitions-----
#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 21
#define RST_PIN 17

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

//LED Library
#include "FastLED.h"                                          // FastLED library. Preferably the latest copy of FastLED 2.1.


//------Pin Definitions-----

//Motor Definitions

const int A1A = 32;//define pin 2 for A1A
const int A1B = 33;//define pin 3 for A1B

//Limit switch definitions

#define InLimitPin 25
#define OutLimitPin 26
#define HandleTriggerPin 27
#define PressureSwitchPin 12

//Handle Light definitions
#define handleLEDpin 14  //led that is built into the handle

//LED Strip pin Definitons
// Fixed global definitions.
#define LED_DT 4                                             // Data pin to connect to the strip.


//-----Integer Definitions-----

//limit switch integers
int InLimitStatus = 0;
int OutLimitStatus = 0;
int HandleTriggerStatus = 0; //storing handle pulled switch status

//Pressure Switch
int PressureSwitchStatus = 0;  //For storing Pressure Sensor state

//Serial definitons
char receivedChar;
boolean newData = false;


//motion state definitions
int currentPosition = 0; //0 means closed 1 means open
int targetPosition = 0; //the handle position we want it to move to



//LED Strip Definitons
#define COLOR_ORDER GRB                                       // Are they RGB, GRB or what??
#define LED_TYPE WS2812B                                       // Don't forget to change LEDS.addLeds
#define NUM_LEDS 10                                           // Number of LED's.
int brightness = 0;  //brigtness used in fade functions
int fadeAmount = 5;  //speed of fade

uint8_t max_bright = 128;                                      // Overall brightness definition. It can be changed on the fly.
struct CRGB leds[NUM_LEDS];                                   // Initialize our LED array.

int LEDstatus = 0;   //0 is off, 1 is locked, 2, is unlocked, 3 is rf read, 4 is motion detected
int LastLEDstatus = 0; //store last led status


//Global State Variables (these are used to update everyting globally)

int LockedState = 0; //State to store whether or not the door is locked
int HandlePulledState = 0;  //0 os no pull, 1 is pulled
int LastHandlePulledState = 0;
int MotionState = 0;  //0 is no motion, 1 is motion
int DoorState = 0; //Whether or not door is closed, 0 is closed, 1 is open



void setup() {
  Serial.begin(115200);

  //Motor Pin Setup
  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);

  //Limit pin setup
  pinMode(InLimitPin, INPUT);
  pinMode(OutLimitPin, INPUT);
  pinMode(HandleTriggerPin, INPUT);

  //Pressure Switch
  pinMode(PressureSwitchPin, INPUT);

  //handle led pin setup
  pinMode(handleLEDpin, OUTPUT);

  //LED Strip Setup
  LEDS.addLeds<LED_TYPE, LED_DT, COLOR_ORDER>(leds, NUM_LEDS);  // Use this for WS2801 or APA102
  set_max_power_in_volts_and_milliamps(5, 500);               // This is used by the power management functionality and is currently set at 5V, 500mA.
  FastLED.setBrightness(max_bright);
  LEDstatus = 3;  //set leds to off
  delay(1000);
  LEDstatus = 0;

  delay(1000);

  //Initializing the handle
  UpdateLimits();
  UpdatePosition();
  if (currentPosition == 0) {
    Serial.println("Handle Closed");
  }
  if (currentPosition == 1) {
    Serial.println("Handle Opened");
  }

  //RFID setup
  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
  Serial.println("Listening for RFID cards ...");
  Serial.println();
  //If you set Antenna Gain to Max it will increase reading distance
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

  //Finishing up setup
  Serial.println("Setup and ready");
}

void loop() {

  //-----Handle Loop-----
  ListenToPressureSensor();

  UpdateLimits();
  //debugSwitches();
  HandleSerial(); //listen for commands for position control


  //Listen for position change commands
  if (targetPosition == 1 && currentPosition == 0) {
    openHandle();
  }

  if (targetPosition == 0 && currentPosition == 1) {
    closeHandle();
  }

  //Listen for handle pull commands
  if (HandleTriggerStatus == 1) {
    HandlePulled(); //runs whatever function needed when handle is pulled
  } else {
    HandleNotPulled();
  }

  //run functions based on handle position
  if (currentPosition == 1) {
    HandleOpen();
  }
  if (currentPosition == 0) {
    HandleClosed();
  }


  //-----RFID Loop-----
  ListenForRFID();

  //Update Status Leds
  HandleIndicatorLEDs();



}//-----end loop-----:)

//------------Handle State Functions--------------

void HandlePulled() {
  Serial.println("Handle Pulled");
  HandlePulledState = 1;
}

void HandleNotPulled() {
  HandlePulledState  = 0;
}

void HandleOpen() {
  digitalWrite(handleLEDpin, HIGH);
}

void HandleClosed() {
  digitalWrite(handleLEDpin, LOW);
}

//------------motor control functions--------------

void openHandle() {
  if (OutLimitStatus == 0) {
    Serial.println("Handle Opening");
    motor('R');// Turn motor on in opening direction
    LEDstatus = 4; //handle is open, update leds
  }

  if (OutLimitStatus == 1) {
    motor('O');// Turn motor OFF
    Serial.println("Handle Open");
    delay(750); //debounce delay for other functions
    currentPosition = 1;  //update current position
  }
}


void closeHandle() {
  if (InLimitStatus == 0) {
    Serial.println("Handle Closing");
    motor('L');// Turn motor on in opening direction
  }

  if (InLimitStatus == 1) {
    motor('O');// Turn motor OFF
    Serial.println("Handle Closed");
    delay(750); //debounce delay for other functions
    currentPosition = 0;  //update current position
  }
}

void motor(char d)
{
  if (d == 'R') {
    digitalWrite(A1A, LOW);
    digitalWrite(A1B, HIGH);
  } else if (d == 'L') {
    digitalWrite(A1A, HIGH);
    digitalWrite(A1B, LOW);
  } else {
    //Robojax.com L9110 Motor Tutorial
    // Turn motor OFF
    digitalWrite(A1A, LOW);
    digitalWrite(A1B, LOW);
  }
}// motorA end



//----------position and limit reading functions-------------

void UpdatePosition() {

  if (OutLimitStatus == 1) {
    currentPosition = 1;  //update current position
  }
  if (InLimitStatus == 1) {
    currentPosition = 0;  //update current position
  }
}

void UpdateLimits() {
  //read limit switches
  InLimitStatus = digitalRead(InLimitPin);
  OutLimitStatus = digitalRead(OutLimitPin);
  HandleTriggerStatus = digitalRead(HandleTriggerPin);
}


void debugSwitches() {
  Serial.print("In Limit- ");
  Serial.print(InLimitStatus);
  Serial.print("---");
  Serial.print("Out Limit- ");
  Serial.print(OutLimitStatus);
  Serial.print("---");
  Serial.print("Handle Switch- ");
  Serial.println(HandleTriggerStatus);
}

bool ReadPressureSensor() {  //returns 1 or 0 based on pulled state
  PressureSwitchStatus = analogRead(PressureSwitchPin);
  //Serial.println(PressureSwitchStatus);
  if (PressureSwitchStatus == 0) {
    return true;
  } else {
    return false;
  }
}

void ListenToPressureSensor() {
  //Pressure Sensor Locking and Opening
  if (ReadPressureSensor() == true) {  //open the handle if pressure sensor is tapped
    if (LockedState == 0) {
      if (currentPosition == 0) {
        targetPosition = 1;
        LEDstatus = 3; //leds blue
      }
    } else {
      Serial.println("Sorry, I'm Locked. Unlock me with your RFID key");
    }
  }
}




//------------Serial control functions------------


void HandleSerial() {
  if (Serial.available() > 0) {
    receivedChar = Serial.read();

    if (receivedChar == 'o') {
      targetPosition = 1;
    }
    if (receivedChar == 'c') {
      targetPosition = 0;
    }
    newData = true;
  }
}


//-----------------RFID Control Functions----------------

void ListenForRFID() {

  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent())
  {
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial())
  {
    return;
  }
  //Show UID on serial monitor
  Serial.print("UID tag :");
  String content = "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  Serial.print("Message : ");
  content.toUpperCase();
  if (content.substring(1) == "59 7D 80 C2") //change here the UID of the card/cards that you want to give access
  {
    Serial.println("Authorized access");
    Serial.println();
    targetPosition = 1;  //Open the handle
    LEDstatus = 2;  //leds green
    //delay(3000);
  }

  else   {
    Serial.println(" Access denied");
    targetPosition = 0; //Close the handle
    LEDstatus = 5; //leds red
    //delay(3000);
  }
}




/**
   Helper routine to dump a byte array as hex values to Serial.
*/
void printHex(byte * buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
   Helper routine to dump a byte array as dec values to Serial.
*/
void printDec(byte * buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }
}


//-------------LED Control Functions-----------
void HandleIndicatorLEDs() {
  if (LEDstatus == 0) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);     // turn leds off
    FastLED.show();
  }
  if (LEDstatus == 1) {  //locked
    fill_solid(leds, NUM_LEDS, CRGB::Orange);     // turn leds off
    FastLED.show();
  }
  if (LEDstatus == 2) {  //unlocked/opened
    fill_solid(leds, NUM_LEDS, CRGB::Green);     // turn leds off;
    FastLED.show();
  }
  if (LEDstatus == 3) {  //rf read
    fill_solid(leds, NUM_LEDS, CRGB::Blue);     // turn leds off
    FastLED.show();
  }
  if (LEDstatus == 4) {  //Motion (white)
    fill_solid(leds, NUM_LEDS, CRGB::White);     // turn leds off
    FastLED.show();
  }
  if (LEDstatus == 5) {  //error
    fill_solid(leds, NUM_LEDS, CRGB::Red);     // turn leds off
    FastLED.show();
  }


  if (HandlePulledState == 1 && LastHandlePulledState == 0) {
    LEDstatus = 3; //blue
    LastHandlePulledState = 1;
  }

  if (HandlePulledState == 0 && LastHandlePulledState == 1) {
    LastHandlePulledState = 0;
    LEDstatus = 0; //off
  }
  
}//end handleledindicators

  void fill_grad() {

    uint8_t starthue = beatsin8(5, 0, 255);
    uint8_t endhue = beatsin8(7, 0, 255);

    if (starthue < endhue) {
      fill_gradient(leds, NUM_LEDS, CHSV(starthue, 255, 255), CHSV(endhue, 255, 255), FORWARD_HUES); // If we don't have this, the colour fill will flip around.
    } else {
      fill_gradient(leds, NUM_LEDS, CHSV(starthue, 255, 255), CHSV(endhue, 255, 255), BACKWARD_HUES);
    }

  } // fill_grad()
