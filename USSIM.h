#define _USE_MATH_DEFINES

//Imports
#include <math.h>
#include <stdbool.h>
#include <Wire.h> //I2C
#include "Adafruit_MCP4725.h" // DAC

// Function Prototypes
void setup();
void loop();

void setup_serial();

void scanning_beam(bool dir, int func);

void reset();

// # Defines

#define AZ 0
#define EL 1
#define BAZ 2

#define SINC_CONST 1.15

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define SCA 4
#define SCL 5

#define TCU_IN 10
#define TCU_CLOCK 11
#define TCU_INT 12

#define BAUD_RATE 9600

#define DAC_ADD 99 // from Datasheet, ADDY set low
// Constant Values

const double FREQ = 78.5 * (10*10*10);
const double OMEGA = 2 * M_PI * FREQ;


// CONST Arrays 

const double AMPLITUDE[7] = \
                          {
                            1,
                            0.5,
                            0.25,
                            10,
                            0,
                            0,
                            0
                          };

const double AMP_SCAN = AMPLITUDE[3];

const double SCAN_BEAM_LIMS[3][2] = \
                            {
                              {
                                -62,
                                62
                              },  
                              {
                                -1.5,
                                29.5
                              },
                              {
                                -42,
                                42
                              }
                            };

const double RECV_POS[3] = \
                        {
                            -10,
                            3,
                            -10
                        };

const double THETA_BW[3] = \
                        {
                            2,
                            1.5,
                            3
                        };

//Global Variables

/*
Used to stop a function at the end. Must be reset on close.
*/
bool STOP_FLAG = false;
Adafruit_MCP4725 dac;
uint8_t SERIAL_IN;

// Lookup Tables

const PROGMEM int16_t DACLookup_Carrier[16] =
{
-1, 83, 152, 194, 203, 177, 120, 42, 
83, 152, 194, 203, 177, 120, 42, -43
};