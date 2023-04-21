/*
USSIM Simulation
UMBC CMPE 349 Spring '23
Group 01
*/

#include "USSIM.h"

void setup() {
    // put your setup code here, to run once:
    // Reset all flags
    reset();
}

void loop() {
  // put your main code here, to run repeatedly:

}


/* 
Configures the serial link to UTCU
  * Only needs to be ran once
  * Needs to resilient in case of failure

PARAMS:
  NONE

RETURNS:
  NONE
*/
void setup_serial() {



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

    // Determine starting position for theta(t)
    double start = SCAN_BEAM_LIMS[func][dir];
    double end = SCAN_BEAM_LIMS[func][!dir];

    // Grab the receiver position, takes value of theta_r
    double theta_r = RECV_POS[func];

    // Grab beamwidth, constant
    double theta_bw = THETA_BW[func];

    // Microseconds since start of scan
    double time;

    // Function of time
    double theta_t;

    // Arguments for sinc pulse
    double x_deg; 
    double x_rad;

    // The output voltage for a given instance of time
    double v;

    // XXX Figure out break sequence
    while (true) {
        // XXX Determine accurate time, in microseconds (us)

        if (dir == 0) {
            theta_t = start + time / 50;
        }
        else {
            theta_t = end - time / 50;
        }

        // Calculate the parameter of sinc, convert to radians from degrees
        x_deg = (theta_t - theta_r) / (SINC_CONST * theta_bw);        
        x_rad = 2 * M_PI * x_rad;

        // Use S23 clarifications equation for voltage
        v = AMP_SCAN * (sin(M_PI * x_rad) / (M_PI * x_rad)) * sin(OMEGA * time);
        
        // Write the voltage value to the DAC
        DAC_write(v);
    }

    reset();
    return;
}


void DAC_write(double val) {

}

void reset() {
    STOP_FLAG = false;
    return;
}