/*
USSIM Simulation
UMBC CMPE 349 Spring '23
Group 01
*/

#include "USSIM.h"

void setup() {
  // put your setup code here, to run once:

  setup_timer();
  setup_dac();

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

  int status = digitalRead(TCU_IN);
  // Add new read to result, equivalent of setting last bit to 0 or 1
  SERIAL_IN = SERIAL_IN + status;

  char print_out[20];
  sprintf(print_out, "Received: %d\n", status);
  //Serial.write(print_out);

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
  if (SERIAL_IN & 0b01000000) {
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
  // select the generic clock generator used as source to the generic clock multiplexer
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset(); //reset TC5

  // Set Timer counter 5 Mode to 16 bits, it will become a 16bit counter ('mode1' in the datasheet)
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 waveform generation mode to 'match frequency'
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler
  //the clock normally counts at the GCLK_TC frequency, but we can set it to divide that frequency to slow it down
  //you can use different prescaler divisons here like TC_CTRLA_PRESCALER_DIV1 to get a different range
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE; //it will divide GCLK_TC frequency by 1
  //set the compare-capture register. 
  //The counter will count up to this value (it's a 16bit counter so we use uint16_t)
  //this is how we fine-tune the frequency, make it count to a lower or higher value
  //F_CPU should be 48 MHZ
  // Trigger every 1 us
  TC5->COUNT16.CC[0].reg = (uint16_t) (F_CPU / 1e6 - 1);
  while (tcIsSyncing());
  
  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing()); //wait until TC5 is done syncing 
}

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until sync'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}


/* 
TC5 Interrupt Handler
*/
void TC5_Handler (void) {
  //YOUR CODE HERE 

  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
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