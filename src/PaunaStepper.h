/*
 *    PaunaStepper.cpp - PaunaStepper, a library (interrupt and timer based) for low-cost stepper motors, such as the 28BYJ-48 5v with ULN2003.
 *    Copyright (c) 2019 Dario Rojas
 * 
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 * 
 *    This permission notice shall be included in all copies or 
 *    substantial portions of the Software.
 * 
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */
 


#ifndef __PAUNA_STEPPER_H__
#define __PAUNA_STEPPER_H__

#include "Arduino.h"

#define _PAUNA_MAX_STEPPER 4 //max steppers motors
#define _pauna_setHighPin(b) ( (b)<8 ? PORTD |=(1<<(b)) : PORTB |=(1<<(b-8)) )
#define _pauna_setLowPin(b) ( (b)<8 ? PORTD &=~(1<<(b)) : PORTB &=~(1<<(b-8)) )

//secuences for halfstep (step8) and fullstep (steps4)
static bool _pauna_steps4[4][4] = {{1, 1, 0, 0}, {0, 1, 1, 0}, {0, 0, 1, 1}, {1, 0, 0, 1}};  //‭C639‬
static bool _pauna_steps8[8][4] = {{1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}};  //8C462319
static unsigned long  _pauna_counttick[_PAUNA_MAX_STEPPER]; // counter for ticks of timer
static unsigned long  _pauna_countstep[_PAUNA_MAX_STEPPER]; // counter of step
static unsigned long  _pauna_limstep[_PAUNA_MAX_STEPPER];   // next limit to count steps
static int   _pauna_state[_PAUNA_MAX_STEPPER];   // state of steppers
static float _pauna_delay[_PAUNA_MAX_STEPPER];   // time delay for each step (in ticks of timers)   
static int   _pauna_seqidx[_PAUNA_MAX_STEPPER];  // index of sequence for half and full step
static int   _pauna_dir[_PAUNA_MAX_STEPPER];     // direction -1, 1 of rotation
static int   _pauna_type[_PAUNA_MAX_STEPPER];    // type of stepper (full o half)
static int   _pauna_odo[_PAUNA_MAX_STEPPER];     // odometry (acumulative steps)
static int   _pauna_pin_motor[_PAUNA_MAX_STEPPER][4];  // pin configuration     
static int   _pauna_freq=0;           // frequency of timer2
static bool  _pauna_isactive=false;   // flag to initialice timer2
static int   _pauna_nmotor=0;         // num of motors attached.


/* States for each stepper
 * P_FOWARD   = move stepper in positive direction of sequence
 * P_BACKWARD = move stepper in negative direction of sequence
 * P_REVERSE  = invert direction of ratation (only in actual move)
 * P_BRAKE    = abort secuence, then change state to BREAKED
 * P_DEATTACHED = pins to stepper are LOW and in HIGH IMPEDANCY (ignoring)
 * P_STOP       = stepper is stop and in float mode (all pins to LOW), then state is change to STOPPED
 * P_STOPPED    = stepper is in float mode
 * P_BRAKED     = stepper is stopped, but pin keep the sequence
 */
enum PAUNA_STATES {P_FORWARD, P_BACKWARD, P_REVERSE, P_BRAKE, P_DEATTACHED, P_STOP, P_STOPPED, P_BRAKED};

/* Frequency configuration for timer2 in Hz*/
enum PAUNA_FREQ   {P_FREQ10000, P_FREQ1000, P_FREQ500, P_FREQ100};

/* Type of secuences */
enum PAUNA_TYPE   {P_FULLSTEP, P_HALFSTEP};


class PaunaStepper {
  public:  
    PaunaStepper();  

    // atach-configure steeper with default parameters = step_delay_x=20, step_per_rev=4096, type = HALFSTEP
    bool attach(int pin1, int pin2, int pin3, int pin4);
    // atach-configure steeper to arduino with parameters
    bool attach(int pin1, int pin2, int pin3, int pin4, int step_delay_x, int step_per_rev, PAUNA_TYPE type);
	// deattach stepper (HIGH impedance in pins)
    bool deattach();

    // no blocking moves
    bool move(PAUNA_STATES mov); // move in direction "mov" indefinelly
    bool moveFor(PAUNA_STATES mov, long time_millis); // move in direction "mov" for "time_millis" seconds
    void rotate(float deg);       // rotate "deg" degrees with deg<0 BACKWARD and deg>0 FORWARD
    void rotateTo(float deg);     // rotate to "deg" degrees with deg<0 BACKWARD and deg>0 FORWARD, use odometry for calculate absolute position
    void rotateStep(long steps);  // rotate "steps" steps with steps<0 BACKWARD and steps>0 FORWARD          
    void rotateStepTo(long steps);// rotate to "steps" steps with steps<0 BACKWARD and steps>0 FORWARD, use odometry for calculate absolute position                      

    // blocking moves, no return until the move ends.
    bool waitMoveFor(PAUNA_STATES mov, long time_millis);
    void waitRotate(float deg);
    void waitRotateTo(float deg);
    void waitRotateFor(float deg, long time_millis);
    void waitRotateStep(long steps);            
    void waitRotateStepFor(long steps, long time_millis);
    void waitRotateStepTo(long steps);            

    // Configure Types and Speed
    bool setStepDelayX(int step_delayx); // set time between steps, depend of PAUNA_FREQ on timer.
    bool setRPM(float rpm);  // set step_delay to reach "rpm" revolution peer minute
    void setOdo(long odometry); //  set odometry (accomulative countstep)   
    void resetOdo();         // reset odometry (accomulative countstep) to 0   

    // Getters
    long   getOdo();         // return odometry (accomulative countstep)
    long   getLimitStep();   // return limit steps to reach in actual move
    long   getCountStep();   // return count steps (relative countstep)
    float  getStepDelayX();  // return time between steps
    int    getMotorId();     // return internal motorid.
    int    getNumMotors();   // return number of stepper attached
    String getStateStr();    // return state of stepper in string format
    bool   isMoving();       // return true if is movig, false if is breaked, stopped o deattached
    PAUNA_STATES getState(); // return state of stepper (PAUNA_STATES)
    PAUNA_TYPE   getType();  // return type of stepper (PAUNA_TYPE)
    PAUNA_FREQ   getInterruptFreq();  // return frequency of timer2

  private:    
    int  _steps_per_revol;   // steps per revolution
    int  _pauna_motorid;     // internal motorid
};

#endif
