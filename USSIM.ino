/*
USSIM Simulation
UMBC CMPE 349 Spring '23
Group 01
*/

#include "USSIM.h"

void setup() {
  // put your setup code here, to run once:

  // Setup Serial
  Serial.begin(BAUD_RATE);

  // Reset all flags
  reset();

  //Pin Statuses
  pinMode(TCU_IN, INPUT);
  pinMode(TCU_CLOCK, INPUT);
  pinMode(TCU_INT, INPUT);

  //Setup Interrupts
  attachInterrupt(digitalPinToInterrupt(TCU_CLOCK), read_tcu_serial, FALLING);
  attachInterrupt(digitalPinToInterrupt(TCU_INT), implement_func, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
}

/*
Reads TCU_IN, stores to next open position in SERIAL_IN

PARAMS: 
  NONE

RETURNS:
  NONE
*/
void read_tcu_serial() {
  // Shift left once for each read
  SERIAL_IN = SERIAL_IN << 1;

  // Add new read to result, equivalent of setting last bit to 0 or 1
  SERIAL_IN = SERIAL_IN + digitalRead(TCU_IN);
  return;
}


/*
Implements TCUs desired function using SERIAL_IN


PARAMS:
  NONE

RETURNS:
  NONE
*/
void implement_func() {
  
  bool scanning_beam = false;

  Serial.write("NEW COMMAND\n");
  // DECODE

  Serial.write("\tTRANSMITTER: ");
  if (SERIAL_IN & 1000000) {
    Serial.write("ON\n");
  }
  else {
    Serial.write("OFF\n");
  }

  Serial.write("\tANTENNA: ");

  switch (SERIAL_IN & 0b00111000) {
    case (0b00000000):
      Serial.write("IDENT/DATA\n");
      break;
    
    case (0b0000100):
      Serial.write("LEFT OCI\n");
      break;

    case(0b00010000):
      Serial.write("REAR/UPPER OCI\n");
      break;
    
    case(0b00011000):
      Serial.write("RIGHT OCI\n");
      break;
    
    case(0b00100000):
      Serial.write("SCANNING BEAM\n");
      scanning_beam = true;
      break;
    
    case(0b00101000):
      Serial.write("OFF\n");
      break;
    
    case(0b00110000):
      Serial.write("TEST\n");
      break;
    
    case(0b00111000):
      Serial.write("UNUSED\n");
      break;
  }

  Serial.write("\tFUNCTION: ");

  if (!scanning_beam) {
    Serial.write("PHASE CHANGE\n");

    //Reset serial input for next function
    SERIAL_IN = 0;
    
    return;
  }

  switch(SERIAL_IN & 0b00000110) {
    case (0b000000000):
      Serial.write("AZIMUTH\n");
      break;
    
    case (0b00000010):
      Serial.write("ELEVATION\n");
      break;
    
    case (0b00000100):
      Serial.write("BACK-AZIMUTH");
      break;
    
    case (0b00000110):
      Serial.write("UNUSED/HRAZ");
      break;
  }

  Serial.write("\nDIRECTION: ");
  switch(SERIAL_IN & 0b00000001) {
    case (0b00000000):
      Serial.write("TO");
      break;
    
    case (0b00000001):
      Serial.write("FRO");
      break;
  }

  //Reset serial input for next function
  SERIAL_IN = 0;

  return;
}


/*
Configues I2C to the DAC
  * Only needs to be ran once

PARAMS:
  NONE

RETURNS:
  NONE
*/
void setup_dac() {
  // Join I2C bus as controller, point towards DAC
  dac.begin(DAC_ADD);
}


/*
Configures TC5
*/
void setup_timer() {
}


/*
Sets up and executes a scanning beam simulation
  * Operation better defined in S23 clarifications

PARAMS:
  bool dir - 0 = to; 1 = fro
  int func - 0 = AZ; 1 = EL; 2 = BAZ

RETURNS:
  NONE
*/
void scanning_beam(bool dir, int func) {

  // BAZ scans backwards, flip direction
  if (func == BAZ) {
    dir = !dir;
  }

  // XXX Implement
}

/*
Sets the output voltage  of the DAC
  * Must be on the interval [-2048, 2047]
  * Does not write to EEPROM

PARAMS:
  int val - 12 bit signed integer value

RETURNS:
  NONE
*/
void DAC_write(int val) {
  dac.setVoltage(val, false);
  return;
}

void reset() {
  STOP_FLAG = false;
  return;
}