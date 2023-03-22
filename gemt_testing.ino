#include "basic_features.h"


/*========================================================================
 TODO:
  - Create a manual servo menu screen - DONE
  - Create a test result screen with "ok" confimrtations - DONE
  - Create updating outputs for ultrasonic testing screen - DONE
  - 
========================================================================*/

static Menu MainMenu[]
{
  {1, "Ultrasonic Test", ultraSonicSensorConnections},
  {2, "9G Servo Menu", servoMenu},
  {3, "ESR Test", esrConnections},
  {4, "nRF24 Test", nrfConnections},
  {5, "L8298N Test", l8298nConnections},
};

size_t mainMenuLen = sizeof(MainMenu) / sizeof(MainMenu[0]);

Menu* CurrentMenuPtr = MainMenu;
size_t* currentMenuLengthPtr = mainMenuLen;

void setup(void)
{
  delay(500);
  Serial.begin(115200); // DEBUGGING
  startInterface();
  
  //ESR Pins
  pinMode(ESR_PIN, INPUT);//reading miliVolt
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DISCHARGE_PIN, OUTPUT);

  //L8298 Pins
  pinMode(L8ENA, OUTPUT);
  pinMode(L8IN1, OUTPUT);
  pinMode(L8IN2, OUTPUT);
  pinMode(L8ENB, OUTPUT);
  pinMode(L8IN3, OUTPUT);
  pinMode(L8IN4, OUTPUT);

  //Ultrasonic Sensor Pins
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //nRF Pins

  //Servo Pins

  
}

int main(void) 
{
  // Must intialize the arduino firmware
  init();
  setup();

  // This where you place whatever needs to be tested
  while(true)
  {
    displayMenu(CurrentMenuPtr, currentMenuLengthPtr);
    delay(3);
  }
  
  return 0;
}
