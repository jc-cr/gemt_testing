#ifndef basic_features_h
#define basic_features_h

//*********************************************************************
// Header for basic utility functions required to test stuff
//*********************************************************************
#include "Arduino.h"
#include <Wire.h>
#include <Servo.h>
#include <HCSR04.h>
#include <RF24.h>
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
void esrTest(void);
void nrfTest(void);
void l8298nTest(void);
long readVcc(void);
void voltL(void);
void voltR(void);
unsigned long measureESR(void);
#define displayRowLimit 8;
#define displayColLimit 21;

/* Use of volatile for variables that:
    - Memory mapped registers
    - global varchanged by interrupt
    - global var accessed by multithreading
    https://barrgroup.com/embedded-systems/how-to/c-volatile-keyword
*/

// Anon namespace better than static
namespace
{
  volatile uint8_t          ebState             = 0; // Current state (Position) of the encoder. Max by uint8 is 255
  volatile bool             clicked             = false; // Updated on encoder "click" case, must reset after use 
  uint8_t                   clickedItemNumber   = 0;
  unsigned short int        currentScrollLimit  = 0;
  const unsigned short int  triggerPin          = 27;
  const unsigned short int  echoPin             = 26;
  const unsigned short int  servoPWMPin         = 9;
  const unsigned short int  DISCHARGE_PIN       = 42;
  const unsigned short int  ESR_PIN             = A0;
  const unsigned short int  PULSE_PIN           = 43;
  const unsigned short int  L8IN1               = 5;
  const unsigned short int  L8IN2               = 6;
  const unsigned short int  L8ENA               = 7;
  const unsigned short int  L8IN3               = 8;
  const unsigned short int  L8IN4               = 9;
  const unsigned short int  L8ENB               = 10;
}


//========================================================================
// Initializers
//========================================================================

// Note: If using jumper wires make sure pins are well spaced out.
// rotary encoder is super noisy and registers false clicks among other issues
// .........CHANGE THESE TO YOUR CURRENT SETUP...................
enum encoderSWPins 
{
  pinA  = 3, // CLK
  pinB  = 2, // DT
  pinSW = 7, // SW
};

enum oledDisplayPins
{
  screenWidth   = 128,
  screenHeight  = 64,
  screenAddress = 0x3C, // i2c Address
  screenReset   = -1      // -1 since sharing Arduino reset pin
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

/*************************************
 * Connection and Menu functions
 *  USS Connections
 *  ESR Connections
 *  NRF Connections
 *  L298N Connections
 *  9G SERVO MENU
 *************************************/

//Connections screen for USS. Will display the connections with an OK confirmation until the user clicks OK, then starts the USS test.
void ultraSonicSensorConnections(void)
{
  while(!clicked)
  {
    eb1.update();
    displayPrep();
    
    display.println("      Connections");
    display.println("+ -> 5V | - -> GND"); display.println("Trigger -> 27");display.println("Echo -> 26");
    
    printConfirm();
    display.display();
  }

  clicked = 0;
  ultraSonicSensor_Test();
}

void esrConnections(void)
{
  while(!clicked)
  {
    eb1.update();
    displayPrep();

    display.println("      Connections");
    display.println("Cathode -> Purple Pin"); display.println("Anode -> A0");
    
    printConfirm();
    display.display();
  }

  clicked = 0;
  esrTest();
}

void nrfConnections(void)
{
  while(!clicked)
  {
    eb1.update();
    displayPrep();

    display.println("      Connections");
    display.println("+ -> 5V | - -> GND"); display.println("enA -> 1 | enB -> 6");display.println("out1 -> 2 | out2 -> 3");

    printConfirm();
    display.println();
  }
  
  clicked = 0;
  nrfTest();
}

void l8298nConnections(void)
{
  while(!clicked)
  {
    eb1.update();
    displayPrep();

    display.println("      Connections");
    display.println("Bottom HBridge ->"); display.println("Bottom PCB Row");display.println("out1-4 -> Left PCB Rw");display.println("Right -> Analog");

    printConfirm();
    display.println();
  }

  clicked = 0;
  l8298nTest();
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

/*****************************
 * TEST FUNCTIONS
 *  Manual Servo Test
 *  Automatic Servo Test
 *  USS Test
 *  ESR Test
 *  NRF Test
 *  L298N Test
 *****************************/

//Function which takes an input from the rotary encoder, then writes the servo to the input. Once the user has their desired angle input, they must click the encoder to write to servo.
void manualServoTest(void)
{
  Servo servo;
  unsigned short int clkPinLast     = LOW;
  unsigned short int clkPinCurrent  = digitalRead(pinA);
  int desiredAngle                  = 0;
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
        if (desiredAngle > 0)
        {
          desiredAngle--;
        }
      }
      else
      {
        if (desiredAngle < 180)
        {
          desiredAngle++;
        }
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

//Function which constantly display the new ESR value of an electrolytic capacitor onto the OLED, until the user clicks the encoder to exit.
void esrTest(void)
{
  #define FASTADC 1
  // defines for setting and clearing register bits
  #ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
  #endif
  #ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
  #endif
  unsigned long esrSamples;
  double        miliVolt;
  double        esrVal;
  double        esrCal;
  double        Vcc;
  double        totalAvg;
  double        vRef = 1.069;//voltage on the Aref pin 
  double        current = 0.088564;
  int           count = 0;
  char          buffer[50];

  Vcc = readVcc(); //sets Vcc to well defined and measured arduino power rail voltage
  analogReference(INTERNAL);//setting vRef to internal reference 1.1V
  digitalWrite(PULSE_PIN,HIGH);//low enables T1
  digitalWrite(PULSE_PIN,HIGH);//low disables T2
  //pinMode(BUTTON_PIN,INPUT_PULLUP);//setting up for a button (will use this for zeroing)
  delay(1000);
  if (FASTADC) 
  {
    sbi(ADCSRA,ADPS2);
    cbi(ADCSRA,ADPS1);
    sbi(ADCSRA,ADPS0);
  }
  //loop to update display with ESR values until user clicks encoder to end test.
  while (!clicked)
  {
    eb1.update();
    displayPrep();

    esrSamples = measureESR();//this function takes a while,)
    miliVolt = (esrSamples * vRef) / 65.535;//calculating voltage on AIN0 pin
    esrVal = 100 / ((Vcc/miliVolt)-1); //esr value in ohms
    //esrVal = (miliVolt*100)/((Vcc)-(miliVolt));
    esrVal = esrVal * 1000; //esrval in mOhms
    esrVal = esrVal - 286.77;
    sprintf(buffer, "ESR: %d", esrVal);
    display.println(buffer);
    printConfirm();
    display.display();
    delay(1000);
  }
  clicked = 0;
}

//oversampler function for measuring ESR value
unsigned long measureESR(void)
{
  unsigned long samples     = 0;
  unsigned int  acumulator  = 0;
  int           i           = 0;
  //oversampling 4096 times (for 16 bit is 4^(desiredResolution - ADCresolution))
  while(i < 4096) {
    digitalWrite(DISCHARGE_PIN,HIGH);//discharge caps
    delayMicroseconds(600);
    digitalWrite(DISCHARGE_PIN,LOW); //disable discharging
    digitalWrite(PULSE_PIN,LOW);//making a miliVolt pulse of 50mA
    delayMicroseconds(5);//on the scope it looks that after enabling the pulse a litle delay is
    //recomended so the oscillations fade away
    acumulator = analogRead(ESR_PIN);//reading value on AIN0
    digitalWrite(PULSE_PIN,HIGH);//stopping pulse
    samples += acumulator;//acumulating the readings
    i++;
  }
  samples = samples >> 6;//decimating value
  return samples;
}

//function designed to find the true Vcc power rail voltage, used for ESR calculations
long readVcc(void) {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void nrfTest(void)
{
  bool result;
  
  //create an RF24 object
  // All test don on SPI2 port input
  
  RF24 spi1(22, 23);  // CE, CSN
  RF24 spi2(24, 25);  // CE, CSN

  spi1.begin();
  spi2.begin();
  
  //address through which two modules communicate.
  const byte address[6] = "00002";
  
  //printHline('#');
  eb1.update();
  displayPrep();
  display.println("nRF24 Test started...");
  display.display();
  //delay(1000);

  // Test connections, display feedback
  if (spi2.isChipConnected() == 1 && spi1.isChipConnected() == 1) 
  {
    // TEST 1: SPI2 TX
    //----------------------------------------------
    //set the address
    spi1.openReadingPipe(0, address);

    // RX Radio config:
    spi1.flush_tx(); // Clear 3 FIFO buffers
    spi1.setPALevel(0); // Set power amplifier low
    spi1.startListening(); //Set module as receiver
    
    // TX Radio config:
    spi2.setPALevel(0); // Set power amplifier min
    spi2.openWritingPipe(address); //set the address
    spi2.stopListening(); //Set module as transmitter

    // Transmission test. Loop few times because radios are weird
    const char text[] = "Test message";
    char msgBuf[32] = {0};
    for (int i = 0; i < 5; ++i)    
    {
      spi1.read(&msgBuf, sizeof(msgBuf));
      
      delay(10);
      
      if (spi2.write(&text, sizeof(text))) 
      { 
        // We got acknoledgement of message, test done
        result = 1;
        break;
      }
      else
      {
         result = 0;
      }
      
      delay(100);
    }

    // If first test passed, continue to next test
    if (result == 1)
    {
      // TEST 2: SPI2 RX
      //----------------------------------------------
    
      // RX radio config:
      spi2.openReadingPipe(0, address); //set the address   
      spi2.flush_tx();  // Clear 3 FIFO buffers  
      spi2.setPALevel(0);  // Set power amplifier low
      spi2.startListening();  //Set module as receiver
    
      // TX Radio config:
      spi1.setPALevel(0);     // Set power amplifier min 
      spi1.openWritingPipe(address); //set the address
      spi1.stopListening(); //Set module as transmitter
  
      // Transmission test. Loop few times because radios are weird
      const char text[] = "Test message";
      char msgBuf[32] = {0};
      for (int i = 0; i < 5; ++i)    
      {
        spi2.read(&msgBuf, sizeof(msgBuf));
        
        delay(10);
        
        if (spi1.write(&text, sizeof(text))) 
        { 
          // We got acknoledgement of message, test done
          result = 1;
          break;
        }
        else
        {
           result = 0;
        }
        
        delay(100);
      }
    }
    displayPrep();
  }
  
  else if (spi2.isChipConnected() == 1 && spi1.isChipConnected() == 0) 
  {
    display.println("SPI1 not detected");
    result = 0;
  }
  
  else if ( spi2.isChipConnected() == 0 && spi1.isChipConnected() == 1)
  {
    display.println("SPI2 not detected");
    result = 0;
  }
  
  else {
    display.println("SPI1 and SPI2");display.println("not detected");
    result = 0;
  }


  // Return final results
  if (result == 1)
  {
    display.println("NRF TEST PASSED");
  }
  else
  {
    display.println("NRF TEST FAILED");
  }
  display.display();
  delay(3000);
}

void l8298nTest(void)
{
  displayPrep();
  display.println("L8298N Test Starting");
  display.display();
  
  digitalWrite(L8IN1, LOW);
  digitalWrite(L8IN2, LOW);
  digitalWrite(L8IN3, LOW);
  digitalWrite(L8IN4, LOW);

  analogWrite(L8ENA, 255);
  digitalWrite(L8IN1, HIGH);
  digitalWrite(L8IN2, LOW);
  delay(2000);
  voltL();
  digitalWrite(L8IN1, LOW);
  digitalWrite(L8IN2, HIGH);
  delay(2000);
  voltL();
  digitalWrite(L8IN1, LOW);
  digitalWrite(L8IN2, LOW);
  
  analogWrite(L8ENB, 255);
  digitalWrite(L8IN3, HIGH);
  digitalWrite(L8IN4, LOW);
  delay(2000);
  voltR();
  digitalWrite(L8IN3, LOW);
  digitalWrite(L8IN4, HIGH);
  delay(2000);
  voltR();

  digitalWrite(L8IN3, LOW);
  digitalWrite(L8IN4, LOW);
  delay(200);
  
}
void voltL(void)
{
  char buffer[50];
  displayPrep();
  int value_in1 = analogRead(A1);
  float voltage_in1 = value_in1 * 5.0/1023;
  sprintf(buffer, "Voltage OUT1 = %f", voltage_in1);
  display.println(buffer);
  int value_in2 = analogRead(A2);
  float voltage_in2 = value_in2 * 5.0/1023;
  sprintf(buffer, "Voltage OUT2 = %f", voltage_in2);
  display.println(buffer);
  display.display();
}
void voltR(void)
{
  char buffer[50];
  displayPrep();
  int value_in3 = analogRead(A3);
  float voltage_in3 = value_in3 * 5.0/1023;
  sprintf(buffer, "Voltage OUT3 = %f", voltage_in3);
  display.println(buffer);
  int value_in4 = analogRead(A4);
  float voltage_in4 = value_in4 * 5.0/1023;
  sprintf(buffer, "Voltage OUT4 = %f", voltage_in4);
  display.println(buffer);
  display.display();
}

/*******************************************
 *  Empty functions for faster debugging
 *******************************************/
/* 
void manualServoTest(void)
{
  
}
void automaticServoTest(void)
{
  
}
void ultraSonicSensor_Test(void)
{
  
}
void esrTest(void)
{
  
}
void nrfTest(void)
{
  
}
void l8298nTest(void)
{
  
}*/


#endif
