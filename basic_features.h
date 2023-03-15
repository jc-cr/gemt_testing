#ifndef basic_features_h
#define basic_features_h

//*********************************************************************
// Header for basic utility functions required to test stuff
//*********************************************************************
#include "Arduino.h"
#include <Wire.h>
#include <Servo.h>
#include <HCSR04.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EncoderButton.h>
#include <Fonts/Org_01.h>
#include "HardwareSerial.h"

//========================================================================
// Global Definitions
//========================================================================
void ultraSonicSensor_Test(void);
void manualServoTest(void);
void automaticServoTest(void);
#define displayRowLimit 8;
#define displayColLimit 21;

/* Use of volatile for variables that:
    - Memormy mapped registers
    - global varchanged by interrupt
    - global var accessed by multithreading
    https://barrgroup.com/embedded-systems/how-to/c-volatile-keyword
*/

// Anon namespace better than static
namespace
{
  volatile uint8_t ebState = 0; // Current state (Position) of the encoder. Max by uint8 is 255
  volatile bool clicked = false; // Updated on encoder "click" case, must reset after use 
  uint8_t clickedItemNumber = 0;
  unsigned short int currentScrollLimit = 0;
  unsigned short int triggerPin = 27;
  unsigned short int echoPin = 26;
  unsigned short int servoPWMPin = 9;
}


//========================================================================
// Initializers
//========================================================================

// Note: If using jumper wires make sure pins are well spaced out.
// rotary encoder is super noisy and registers false clicks among other issues
// .........CHANGE THESE TO YOUR CURRENT SETUP...................
enum encoderSWPins 
{
  pinA = 3, // CLK
  pinB = 2, // DT
  pinSW = 7, // SW
};

enum oledDisplayPins
{
  screenWidth = 128,
  screenHeight = 64,
  screenAddress = 0x3C, // i2c Address
  screenReset = -1      // -1 since sharing Arduino reset pin
};

// Display init
Adafruit_SSD1306 display(screenWidth, screenHeight, &Wire, screenReset);

// EncoderSW init
EncoderButton eb1(pinA, pinB, pinSW);

// Menu screen template
typedef struct Menu
{
  unsigned int choice;
  const char* menuTextPtr;
  void (*selectionAction)(...); // Function pointer to Menu selection action
  //ArgList *head;

  //void setSelectionParams(...);
  //void runSelectionAction(...);
  // Method styling in C https://www.cs.uaf.edu/courses/cs301/2014-fall/notes/methods/index.html
  // Ellipses ref https://www.lemoda.net/c/function-pointer-ellipsis/
} Menu;

//========================================================================
// Encoder Handlers (Interrupt functions)
//========================================================================

// On click, the global selection variable gets updated with
// value of where it was selcted
void onEb1Clicked(EncoderButton& eb)
{
  // Set selection value to current state
  clicked = true;
  //clickedItemNumber = CurrentMenuPtr[ebState].choice;

  // DEBUG - Delete in actua\l proram as Serial printing slows down interrupts
  //Serial.println("CLICKED!");
}

// A function to handle the 'encoder' event
void onEb1Encoder(EncoderButton& eb) 
{
  
  // Filter latge spikes from noise
  if(eb.increment() > 4)
  {
    eb.resetPosition(eb.position()); // Reset back to startin pos
  }
  
  // Reset if encoder goes past active Menu limit
  
  if (abs(eb.position()) >= currentScrollLimit)
  {
    eb.resetPosition(0);
  }
  

  ebState = abs(eb.position());

  // DEBUG - Delete in actual proram as Serial printing slows down interrupts
  //Serial.println(ebState);
}
//========================================================================
// Screen Display Functions
//========================================================================


// Modified bootup fucnction for display
void startInterface(void) 
{ 
  if(!display.begin(SSD1306_SWITCHCAPVCC, screenAddress)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  //Link the event(s) to your function
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
  display.setTextSize(1);
  display.clearDisplay();  
  display.display();
}

void displayPrep(void)
{
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
}

void displayMenu(Menu CurrentMenu[], size_t menuLength)
{
  if (clicked)
  {
    clicked = 0;
    CurrentMenu[ebState].selectionAction();
  }
  else
  {
    currentScrollLimit = menuLength;
    char buffer[50];
    eb1.update();
    displayPrep();
    

    display.println("Select Module:");

    for(size_t i = 0; i <= (menuLength - 1); i++)
    {
      if (ebState == i)
      {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
      }
      else
      {
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); 
      }

      sprintf(buffer, "%d. %s", CurrentMenu[i].choice, CurrentMenu[i].menuTextPtr);
      display.println(buffer);
    }
    display.display();
  }
}

//Prints an OK button at the bottom left of the display, to be used with a !clicked loop.
void printConfirm(void)
{
  currentScrollLimit = 1;
  display.setCursor(0,50);
  if (ebState == 0)
  {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  }
  else
  {
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); 
  }
  display.println("OK");
}

//Connections screen for USS. Will display the connections with an OK confirmation until the user clicks OK, then starts the USS test.
void ultraSonicSensorConnections(void)
{
  while(!clicked)
  {
    eb1.update();

    displayPrep();
    display.println("      Connections");
    display.println("+ -> 5V | - -> GND"); display.println("Trigger -> 27");display.println("Echo -> 26");
    display.println();
    
    printConfirm();
    display.display();
  }

  clicked = 0;
  ultraSonicSensor_Test();
  
}

void servoMenu(void)
{
  displayPrep();
  currentScrollLimit = 3;

  static Menu sMenu[] =
  {
    {1, "Manual Test", manualServoTest},
    {2, "Automatic Test", automaticServoTest},
  };

  size_t sMenuLength = sizeof(sMenu) / sizeof(sMenu[0]);

  while (true)
  {
    displayMenu(sMenu, sMenuLength);

    if (clicked)
    {
      displayMenu(sMenu, sMenuLength);
      break;
    }
    delay(3); 
  }
  clicked = 0;

  /*char* options[3] = {"Manual Servo", "Automatic Servo", "BACK"};
  size_t optionsSize = sizeof(options) / sizeof(options[0]);
  for (size_t i = 0; i < (optionsSize - 1); i++)
  {
    
  }*/
  
}

//Function which takes an input from the rotary encoder, then writes the servo to the input. Once the user has their desired angle input, they must click the encoder to write to servo.
//TODO: Limit input between 0 and 180 so the user can't break the servo.
void manualServoTest(void)
{
  Servo servo;
  unsigned short int clkPinLast = LOW;
  unsigned short int clkPinCurrent = digitalRead(pinA);
  int desiredAngle = 0;
  char buffer[50];
  servo.write(0);
  servo.attach(servoPWMPin);
  
  
  while(!clicked)
  {
    eb1.update();
    displayPrep(); 
    display.print("Desired Angle: ");
    if (clkPinLast == LOW && clkPinCurrent == HIGH)
    {
      if (digitalRead(pinB) == HIGH)
      {
        desiredAngle--;
      }
      else
      {
        desiredAngle++;
      }
    }
    display.print(desiredAngle);
    clkPinLast = clkPinCurrent;
    display.display();
  }
  clicked = 0;

  displayPrep();
  servo.write(desiredAngle);
  sprintf(buffer, "Current Angle: %d ", servo.read());
  display.println(buffer);
  display.display();
  delay(2000);
  servo.detach();
}

//Function which increments the servo by 10 degrees to and from 180 degrees. The current angle is updated on the display every 500ms.
void automaticServoTest(void)
{
  Servo servo;
  servo.write(0);
  servo.attach(servoPWMPin);
  char buffer[50];
  int angle = 0;

  displayPrep();
  display.println("Servo will now rotate");display.println("to 180 degrees in 10");display.println("degree increments");
  display.display();
  delay(5000);
  
  for (int i = 0; i <= 18; ++i)
  {
    displayPrep();
    angle = (10 * i);
    sprintf(buffer, "Current Angle: %d", angle);
    display.println(buffer);
    servo.write(angle);
    display.display();
    delay(500);
  }
  
  displayPrep();
  display.println("Servo will now rotate");display.println("to 0 degrees in 10");display.println("degree increments");
  delay(5000);
  display.display();
  angle = 0;
  
  for (int i = 0; i <= 18; ++i)
  {
    displayPrep();
    angle = (180 - (10 * i));
    sprintf(buffer, "Current Angle: %d", angle);
    display.println(buffer);
    servo.write(angle);
    display.display();
    delay(500);
  }
  clicked = 0;
  servo.detach();
}

//Function which constantly updates a Distance onto the OLED, until the user clicks the encoder to exit.
void ultraSonicSensor_Test(void)
{ 
  long      duration;
  int       distance;
  //bool      measuring = true;
  double    permDistance;
  double    samples;
  char      buffer[50]; // init buffer of 50 bytes to hold expected string size
  attachInterrupt(digitalPinToInterrupt(7), onEb1Clicked, RISING); // This is necessary to allow the user to return from test screens, the digitial pin to interrupt is the SWITCH pin. The EncoderButton.h header doesn't automatically do this for some reason.
  while(!clicked)
  {
    eb1.update();
    displayPrep();
    
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    delay(500);
    sprintf(buffer, "Distance: %d cm", distance);
    display.println(buffer);
    printConfirm();
    display.display();
  }
  clicked = 0;
}


#endif
