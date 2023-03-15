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
};

size_t mainMenuLen = sizeof(MainMenu) / sizeof(MainMenu[0]);

Menu* CurrentMenuPtr = MainMenu;
size_t* currentMenuLengthPtr = mainMenuLen;

void setup(void)
{
  delay(500);
  Serial.begin(115200); // DEBUGGING
  startInterface();
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
