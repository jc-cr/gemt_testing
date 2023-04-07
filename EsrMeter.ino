//#include <LiquidCrystal595.h>
#include <avr/eeprom.h>

//we have to change prescaler for the ADC to make the conversion happen faster
#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//define the input and output pin we will use
#define DISCHARGE_PIN 7
#define ESR_PIN A0
#define PULSE_PIN 9
#define BUTTON_PIN 4

//function prototype
unsigned long measureESR(void);//measuring function, increases ADC to 16bit resolution through oversampling

//global variables
unsigned long esrSamples;
double miliVolt;
double esrVal;
double esrCal;
double Vcc;
double vRef = 1.069;//voltage on the Aref pin 
double current = 0.088564;// milliamps proper calibration can be done entering the right value for the current (U=I*R)0.0731
//idealy this is 0.05 A, this condition is fulfilled only if R10 is 100 Ohm, Vcc is exactly 5V and the transistor
//while fully saturated idealy is at 0 ohm.



void setup(void)
{
  Serial.begin(9600);
  Serial.println("ESR Meter");
  Serial.println("Setting up");
  
  Vcc = readVcc(); //sets Vcc to well defined and measured arduino power rail voltage
  Serial.println(Vcc);
  analogReference(INTERNAL);//setting vRef to internal reference 1.1V
 

  pinMode(ESR_PIN, INPUT);//reading miliVolt
  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN,HIGH);//low enables T1
  pinMode(DISCHARGE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN,HIGH);//low disables T2
  pinMode(BUTTON_PIN,INPUT_PULLUP);//setting up for a button (will use this for zeroing)
  //digitalWrite(BUTTON_PIN,HIGH);//enabling the pull up on the button, when button pressed to the ground zeroes out the cable
  delay(1000);
  Serial.println("Please wait...");

  //seting prescaller to 32 for faster adc (500khz)
  //at 500khz  results are still looking good (same values as if 250khz ADC clock)
  // the shorter the pulse on a small value capacitor it has no time to charge and denaturate de result
  if (FASTADC) {
    sbi(ADCSRA,ADPS2);
    cbi(ADCSRA,ADPS1);
    sbi(ADCSRA,ADPS0);
  }

  //reading calibration value, it will be ok if already calibrated, else it might be bogus depends on the content of EEPROM
  //but will be ok after first calibration
 //eeprom_read_block((void*)&esrCal, (void*)0, sizeof(esrCal));
}

void loop(void)
{
  //lcd.setLED1Pin(HIGH);
  esrSamples = measureESR();//this function takes a while,)
  // so we don't need other delay for the lcd (this functions time gives the refresh rate for display
  miliVolt = (esrSamples * vRef) / 65.535;//calculating voltage on AIN0 pin
  esrVal = 100 / ((Vcc/miliVolt)-1); //esr value in ohms
  //esrVal = (miliVolt*100)/((Vcc)-(miliVolt));
  esrVal = esrVal * 1000; //esrval in mOhms
  
  //esrVal = esrVal + 25;
  
  Serial.print("V: ");
  Serial.print(miliVolt);
  Serial.println("mV");
  Serial.print("ESR: ");
  Serial.print(esrVal);
  Serial.println("m");

  
  

  //for zeroing the cables, this can be quite a big resistance compared to the values we intend to measure
  //so it is a good idea to try to reduce in any way possible this influence (short cables, soldering the cables, etc)
  /**if(!digitalRead(BUTTON_PIN)){
    Serial.println("Zeroing...");
    esrCal = (miliVolt)/current; //esrCal is mOhms
    //esrCal = 5350;
    Serial.println("Done");
    Serial.print("Calib in mOhms: ");
    Serial.println(esrCal);
    //writing calibration value into EEPROM so we don't have to calibrate on restart
    eeprom_write_block((const void*)&esrCal, (void*)0, sizeof(esrCal));
    Serial.println("saved to EEPROM");
    delay(400);
  }
  */
}


unsigned long measureESR()
{
  unsigned long samples = 0;
  unsigned int acumulator = 0;
  int i = 0;
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
  //we have samples, let's go and compute value
  samples = samples >> 6;//decimating value
  return samples;//all done returning sampled value
}

long readVcc() {
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
