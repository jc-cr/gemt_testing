#ifndef basic_features_h
#define basic_features_h

//*********************************************************************
// Header for basic utility functions required to test stuff
//*********************************************************************
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EncoderButton.h>
#include <Fonts/Org_01.h>
#include "HardwareSerial.h"

//========================================================================
// Global Definitions
//========================================================================

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
}

//========================================================================
// Initializers
//========================================================================

// Note: If using jumper wires make sure pins are well spaced out.
// rotary encoder is super noisy and registers false clicks among other issues
enum encoderSWPins 
{
  pinA = 19 , // CLK
  pinB = 2, // DT
  pinSW = 38 // SW
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
  /*
  // Filter latge spikes from noise
  if(eb.increment() > 4)
  {
    eb.resetPosition(eb.position()); // Reset back to startin pos
  }
  */
  // Reset if encoder goes past active Menu limit
  /*
  if (abs(eb.position()) >= currentMenuLenPtr)
  {
    eb.resetPosition(0);
  }
  */

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
    display.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  //Link the event(s) to your function
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
  
  display.clearDisplay();  
  display.display();
}



#endif