#include "basic_features.h"


/*========================================================================
 TODO:
  - Create a manual servo menu screen
  - Create a test result screen with "ok" confimrtations
  - Create updating outputs for ultrasonic testing screen
  - 
========================================================================*/

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
    
    delay(3);
  }
  
  return 0;
}