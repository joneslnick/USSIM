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
  delay(300);


  setupDac();

  //Pin Statuses
  pinMode(TCU_IN, INPUT);
  pinMode(TCU_CLOCK, INPUT);
  pinMode(TCU_INT, INPUT);

  pinMode(TEST_PIN, INPUT);

  SERIAL_IN = 0;

  //Setup Interrupts
  attachInterrupt(digitalPinToInterrupt(TCU_CLOCK), readTCUSerial, RISING);
  attachInterrupt(digitalPinToInterrupt(TCU_INT), implementFunction, FALLING);

}


void loop() {
  // put your main code here, to run repeatedly:

  uint16_t next_val = 0;

  //If scanning beam is active
  if (ANTENNA == ANTENNA_SB && SB) {
    next_val = pgm_read_word(&(DACLookup_SB[SB_FUNCTION][SB_INDEX]));

    if (SB_DIRECTION == TO) {
      SB_INDEX = (SB_INDEX + 1) % SB_INDEX_MAX;
    }
    else {
      
      if (SB_INDEX == 0) {
        SB_INDEX = SB_INDEX_MAX;
      }
      
      SB_INDEX = (SB_INDEX - 1);
    }

    /*
    Serial.print("SB INDEX: ");
    Serial.println(SB_INDEX);
    */

    delayMicroseconds(SB_WAIT_TIME);
  }
  //Modulated/Unmodulated Carrier
  else {
    next_val = pgm_read_word(&(DACLookup_Carrier[ANTENNA][CARRIER_INDEX]));  

    CARRIER_INDEX = (CARRIER_INDEX + 1) % CARRIER_INDEX_MAX;
    delayMicroseconds(2);
  }
  
  /*
  uint16_t dac_val = analogRead(TEST_PIN);
  Serial.println(dac_val);
  */

  //Prevent underflow/overflow
  if (next_val > 4096) {
    next_val = ZERO_VAL;
  }

  if (TRANSMITTER_ON) {
    dacWrite(next_val);
  }
  else {
    dacWrite(ZERO_VAL);
  }

}



/*
Reads TCU_IN, stores to next open position in SERIAL_IN

PARAMS: 
  NONE

RETURNS:
  NONE
*/
void readTCUSerial() {
  // Shift left once for each read
  SERIAL_IN = SERIAL_IN << 1;

  bool status = digitalRead(TCU_IN);

  // Add new read to result, equivalent of setting last bit to 0 or 1
  SERIAL_IN = SERIAL_IN + status;
  return;
}


/*
Implements TCUs desired function using SERIAL_IN
  * Follows EFCL given protocl

PARAMS:
  NONE

RETURNS:
  NONE
*/
void implementFunction() {

  // First bit - Transmitter On/OFF
  // If transmitter is off, must be pause
  // Return if transmitter is off
  SB = false;
    
  if (SERIAL_IN & 0b10000000) {
    TRANSMITTER_ON = true;
  }
  else {
    TRANSMITTER_ON = false;

    SERIAL_IN = 0;
    return;
  }

  // 3 bits denote antenna type in binary
  ANTENNA = (SERIAL_IN & 0b00001110) >> 1;


  // Second bit - DPSK
  // If DPSK, then cannot be scanning beam
  if (SERIAL_IN & 0b01000000) {
    changePhase();

    SERIAL_IN = 0;
    return;
  }



  //Must be scanning beam  
  if (ANTENNA == ANTENNA_SB) {
    SB = true;
    TRANSMITTER_ON = true;
    SB_INDEX = 0;
  }
  else {
    SB = false;
    SERIAL_IN = 0; //OCI
    return;
  }

  //Determine direction / starting index
  if (SERIAL_IN & 0b00100000) {
    SB_DIRECTION = TO;
  }
  else {
    SB_DIRECTION = FRO;
  }

  switch (SB_FUNCTION) {
      case (AZ):
        SB_INDEX_MAX = AZ_INDEX_MAX;
        SB_WAIT_TIME = AZ_WAIT_TIME;
        break;
      case (EL):
        SB_INDEX_MAX = EL_INDEX_MAX;
        SB_WAIT_TIME = EL_WAIT_TIME;
        break;
      case (BAZ):
        SB_INDEX_MAX = BAZ_INDEX_MAX;
        SB_WAIT_TIME = BAZ_WAIT_TIME;
        break;
      default:
        SB_INDEX_MAX = AZ_INDEX_MAX;
        SB_WAIT_TIME = AZ_WAIT_TIME;
        break;
  }

  //Flip direction for BAZ
  if (SB_FUNCTION == BAZ) {
    SB_DIRECTION = !SB_DIRECTION;
  }

  if (SB_DIRECTION == TO) {
    SB_INDEX = 0;
  }
  else {
    SB_INDEX = SB_INDEX_MAX - 1;
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
void setupDac() {
  // Join I2C bus as controller, point towards DAC
  dac.begin(0x62);
}



/*
Sets the output voltage  of the DAC
  * Must be on the interval [0, 4095]
  * Does not write to EEPROM

PARAMS:
  uint16_t val - 12 bit signed integer value

RETURNS:
  NONE
*/
void dacWrite(uint16_t val) {
  Serial.println(val);
  dac.setVoltage(val, false);
  return;
}

/*
Performs a phase change in the carrier signal
  * Change CARRIER_INDEX half a period away

PARAMS:
  NONE

RETURNS:
  NONE
*/
void changePhase(){
  CARRIER_INDEX = ((CARRIER_INDEX) + (CARRIER_INDEX_MAX / 2)) % (CARRIER_INDEX_MAX);
  return;
}


