//Imports
#include <stdbool.h>
#include <Adafruit_MCP4725.h> // DAC
#include <Wire.h> // I2C

#include "LookupTables.h"

// # Defines
#define AZ 0
#define EL 1
#define BAZ 2

#define TO 1
#define FRO 0

// Connection to TCU Pinout
#define TCU_IN 10
#define TCU_CLOCK 9
#define TCU_INT 11

#define ZERO_VAL 2048

#define ANTENNA_SB 5

// Debug Line BAUD
#define BAUD_RATE 115200

#define DAC_ADD 0x62 // from Datasheet, ADDY set low


/* 
CHANGE THIS TO CHANGE THE FUNCTION USED FOR SB
0 - AZ
1 - EL
2 - BAZ
*/
#define SB_FUNCTION 1



#define TEST_PIN A5



#define SB_WAIT_TIME 4750

#define AZ_INDEX_MAX 1240
#define EL_INDEX_MAX 310
#define BAZ_INDEX_MAX 840


// Function Definitions
void setup();
void loop();

void readTCUSerial();
void implementFunction();

void setupDac();

void dacWrite(uint16_t val);
void changePhase();


//Global Variables

Adafruit_MCP4725 dac;
volatile uint8_t SERIAL_IN = 0;
volatile uint8_t CARRIER_INDEX = 0;
volatile uint16_t SB_INDEX = 0;
volatile uint16_t SB_INDEX_MAX = 1240;
volatile bool TRANSMITTER_ON = false;

volatile bool SB = false;
volatile bool SB_DIRECTION = true; //True = TO; FALSE = FRO;

volatile uint8_t ANTENNA = 0;

