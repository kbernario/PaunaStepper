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
 
#include <avr/interrupt.h>
#include "PaunaStepper.h"
static inline void pauna_stepInterrupt() {
  cli();
  for (int i=0; i<_PAUNA_MAX_STEPPER; i++) {
    if (_pauna_state[i]!=P_DEATTACHED && _pauna_state[i]!=P_STOPPED && _pauna_state[i]!=P_BRAKED) {
      if (_pauna_counttick[i] >= _pauna_delay[i]) {          
        _pauna_counttick[i]=1;
        switch (_pauna_state[i]) {
          case P_BRAKE:
             _pauna_state[i] = P_BRAKED;
             
             break;
          case P_STOP:
             _pauna_setLowPin(_pauna_pin_motor[i][0]);
             _pauna_setLowPin(_pauna_pin_motor[i][1]);
             _pauna_setLowPin(_pauna_pin_motor[i][2]);
             _pauna_setLowPin(_pauna_pin_motor[i][3]);
             _pauna_state[i] = P_STOPPED;
             break;         
          default:
            if (_pauna_limstep[i]==0 || _pauna_countstep[i]<_pauna_limstep[i]){
              _pauna_seqidx[i] += _pauna_dir[i];
              _pauna_odo[i] += _pauna_dir[i];
              
              if (_pauna_seqidx[i] >= _pauna_type[i]) _pauna_seqidx[i]=0;
              if (_pauna_seqidx[i] < 0) _pauna_seqidx[i] = _pauna_type[i]-1;

              if (_pauna_type[i] == 4) {
                (_pauna_steps4[_pauna_seqidx[i]][0]) ? _pauna_setHighPin(_pauna_pin_motor[i][0]) : _pauna_setLowPin(_pauna_pin_motor[i][0]);
                (_pauna_steps4[_pauna_seqidx[i]][1]) ? _pauna_setHighPin(_pauna_pin_motor[i][1]) : _pauna_setLowPin(_pauna_pin_motor[i][1]);
                (_pauna_steps4[_pauna_seqidx[i]][2]) ? _pauna_setHighPin(_pauna_pin_motor[i][2]) : _pauna_setLowPin(_pauna_pin_motor[i][2]);
                (_pauna_steps4[_pauna_seqidx[i]][3]) ? _pauna_setHighPin(_pauna_pin_motor[i][3]) : _pauna_setLowPin(_pauna_pin_motor[i][3]);
              } else {
                (_pauna_steps8[_pauna_seqidx[i]][0]) ? _pauna_setHighPin(_pauna_pin_motor[i][0]) : _pauna_setLowPin(_pauna_pin_motor[i][0]);
                (_pauna_steps8[_pauna_seqidx[i]][1]) ? _pauna_setHighPin(_pauna_pin_motor[i][1]) : _pauna_setLowPin(_pauna_pin_motor[i][1]);
                (_pauna_steps8[_pauna_seqidx[i]][2]) ? _pauna_setHighPin(_pauna_pin_motor[i][2]) : _pauna_setLowPin(_pauna_pin_motor[i][2]);
                (_pauna_steps8[_pauna_seqidx[i]][3]) ? _pauna_setHighPin(_pauna_pin_motor[i][3]) : _pauna_setLowPin(_pauna_pin_motor[i][3]);                
              }
              if (_pauna_limstep[i]>0) _pauna_countstep[i]++;
            } else {               
               _pauna_state[i] = PUNA_LIMIT_END;
            }
         }
      } else {
        _pauna_counttick[i]++;
      }    
    }
  }
  sei();  
}

static int pauna_getIndex(){
  int i;
  for (i=0; i<_PAUNA_MAX_STEPPER; i++){
    if (_pauna_state[i]==P_DEATTACHED)
      break;
  }

  if (i<_PAUNA_MAX_STEPPER) {
    return i;
  }
  return -1;
}

static void pauna_setLimit(int motorid, unsigned long limit){
  _pauna_limstep[motorid] = limit;
  if (limit>0) {
    _pauna_countstep[motorid]=0;
  }  
}

static void pauna_move(int motorid, PAUNA_STATES mov) {  
  _pauna_state[motorid] = mov;

  switch (mov) {
    case P_STOP:
    case P_STOPPED:
    case P_BRAKE:
    case P_BRAKED:
      _pauna_dir[motorid] = 0;
      break;
    case P_FORWARD:
      _pauna_dir[motorid] = 1;
      break;
    case P_BACKWARD:
      _pauna_dir[motorid] = -1;
      break;
    case P_REVERSE:
      _pauna_dir[motorid] = -_pauna_dir[motorid];
      break;
  }
}

static void pauna_setupTimer(PAUNA_FREQ freq){ 
  _pauna_freq = freq;  
  cli();  
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  switch (freq) {
    case P_FREQ10000:
      // 10000 Hz (16000000/((24+1)*64))
      OCR2A = 24;
      // CTC
      TCCR2A |= (1 << WGM21);
      // Prescaler 64
      TCCR2B |= (1 << CS22);
      break;
    case P_FREQ1000:
      // 1000 Hz (16000000/((124+1)*128))
      OCR2A = 124;
      // CTC
      TCCR2A |= (1 << WGM21);
      // Prescaler 128
      TCCR2B |= (1 << CS22) | (1 << CS20);
      break;
    case P_FREQ500:
      // 500 Hz (16000000/((124+1)*256))
      OCR2A = 124;
      // CTC
      TCCR2A |= (1 << WGM21);
      // Prescaler 256
      TCCR2B |= (1 << CS22) | (1 << CS21);
      // Output Compare Match A Interrupt Enable
      break;
    case P_FREQ100:
      // 100.16025641025641 Hz (16000000/((155+1)*1024))
      OCR2A = 155;
      // CTC
      TCCR2A |= (1 << WGM21);
      // Prescaler 1024
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
      break;
  }
  
  // Output Compare Match A Interrupt Enable
  TIMSK2 |= (1 << OCIE2A);
  sei();
}

static void pauna_attach(int motorid, int p1, int p2, int p3, int p4, int step_delay, PAUNA_TYPE type){  
  if (type == P_FULLSTEP)
    _pauna_type[motorid] = 4;
  else
    _pauna_type[motorid] = 8;
  
  _pauna_pin_motor[motorid][0] = p1;
  _pauna_pin_motor[motorid][1] = p2;
  _pauna_pin_motor[motorid][2] = p3;
  _pauna_pin_motor[motorid][3] = p4;

  _pauna_state[motorid] = P_STOPPED;
  _pauna_seqidx[motorid] = 0;
  _pauna_dir[motorid]   = 0;
  _pauna_countstep[motorid] = 0;
  _pauna_counttick[motorid] = 0;
  _pauna_delay[motorid] = step_delay;
  _pauna_limstep[motorid] = 0;
  _pauna_odo[motorid]=0;
  
  for (int i=0; i<4; i++) {
    digitalWrite(_pauna_pin_motor[motorid][i], LOW);
    pinMode(_pauna_pin_motor[motorid][i], OUTPUT);
  }    

  if (!_pauna_isactive) {
    _pauna_isactive=true;
    pauna_setupTimer(P_FREQ10000);        
  }    
}


ISR(TIMER2_COMPA_vect){
  pauna_stepInterrupt();
}

PaunaStepper::PaunaStepper() {
  if (!_pauna_isactive) {
    for (int i=0; i<_PAUNA_MAX_STEPPER; i++) 
      _pauna_state[i] = P_DEATTACHED;    
  }    
  _pauna_motorid=-1;
}

PAUNA_FREQ PaunaStepper::getInterruptFreq() {
  return _pauna_freq;
}

void PaunaStepper::resetOdo(){
  _pauna_odo[_pauna_motorid]=0;
}

void PaunaStepper::setOdo(long odometry){
  _pauna_odo[_pauna_motorid]=odometry;
}

long PaunaStepper::getOdo(){
  return _pauna_odo[_pauna_motorid];
}

long PaunaStepper::getLimitStep(){
  return _pauna_limstep[_pauna_motorid];
}

long PaunaStepper::getCountStep(){
  return _pauna_countstep[_pauna_motorid];
}

float PaunaStepper::getStepDelayX(){  
  return _pauna_delay[_pauna_motorid];
}

PAUNA_STATES PaunaStepper::getState(){
  return _pauna_state[_pauna_motorid];
}

PAUNA_TYPE PaunaStepper::getType(){
  return _pauna_type[_pauna_motorid];
}

bool PaunaStepper::deattach(){
  if (_pauna_state[_pauna_motorid] != P_DEATTACHED) {
    _pauna_state[_pauna_motorid] = P_DEATTACHED;    
    for (int i = 0; i < 4; i++) {
      digitalWrite(_pauna_pin_motor[_pauna_motorid][i], LOW);
      pinMode(_pauna_pin_motor[_pauna_motorid][i], INPUT);
    }    
    _pauna_nmotor--;    
    return true;
  }
  return false;  
}

bool PaunaStepper::attach(int pin1, int pin2, int pin3, int pin4, int step_delay_x, int step_per_rev, PAUNA_TYPE type){
   _pauna_motorid = pauna_getIndex();
   if (_pauna_motorid>=0 && step_per_rev>0 && (type==P_HALFSTEP || type==P_FULLSTEP)) {
      _pauna_nmotor++;
      _steps_per_revol = step_per_rev;   
      pauna_attach(_pauna_motorid, pin1, pin2, pin3, pin4, step_delay_x, type);
      return true;
   } else 
      return false;
}

bool PaunaStepper::attach(int p1, int p2, int p3, int p4){   
   _pauna_motorid = pauna_getIndex();
   if (_pauna_motorid>=0) {
      _pauna_nmotor++;
      _steps_per_revol = 4096;   
      pauna_attach(_pauna_motorid, p1, p2, p3, p4, 40, P_HALFSTEP);
      return true;
   } else
      return false;
}

int PaunaStepper::getNumMotors() {
      return _pauna_nmotor;
}


void PaunaStepper::waitRotateStep(long limit) {
  rotateStep(limit);
  while(isMoving());  
}

void PaunaStepper::waitRotateStepTo(long limit) {
  rotateStepTo(limit);
  while(isMoving());  
}

void PaunaStepper::waitRotateStepFor(long limit, long time) {
  rotateStep(limit);
  delay(time);  
}

void PaunaStepper::rotateStepTo(long limit) {
  long dif = limit-getOdo();
  rotateStep(dif);
}

void PaunaStepper::rotateStep(long limit) {
  if (limit!=0) {
    pauna_setLimit(_pauna_motorid, abs(limit));
    (limit > 0) ? pauna_move(_pauna_motorid, P_FORWARD) : pauna_move(_pauna_motorid, P_BACKWARD);
  } else
    pauna_move(_pauna_motorid, P_BRAKE);
}

void PaunaStepper::waitRotate(float deg) {
  rotate(deg);
  while(isMoving());  
}

void PaunaStepper::waitRotateTo(float deg) {
  rotateTo(deg);
  while(isMoving());  
}

void PaunaStepper::waitRotateFor(float deg, long time) {
  rotate(deg);
  delay(time);
}

void PaunaStepper::rotateTo(float deg) {
  float dif = deg - ((getOdo() % _steps_per_revol)/(float)_steps_per_revol)*360.0;
  rotate(dif);
}

void PaunaStepper::rotate(float deg) {
  double degstep = 360.0 / _steps_per_revol;
  unsigned long limit = abs(deg) / degstep;
  if (limit!=0) {
    pauna_setLimit(_pauna_motorid, limit);
    (deg >= 0) ? pauna_move(_pauna_motorid, P_FORWARD) : pauna_move(_pauna_motorid, P_BACKWARD);
  } else
    pauna_move(_pauna_motorid, P_BRAKE);
}

bool PaunaStepper::waitMoveFor(PAUNA_STATES mov, long time){
  if (move(mov)) {
    delay(time);
    move(P_BRAKE);
    return true;
  }
  return false;
}

bool PaunaStepper::moveFor(PAUNA_STATES mov, long time){  
  int freq;
  double res;
  if (time > 0) {
    switch (mov) {
      case P_STOP:
      case P_BRAKE:
      case P_FORWARD:
      case P_BACKWARD:
      case P_REVERSE:      
          switch (_pauna_freq) {
            case P_FREQ10000:
              freq=10000;
              break;
            case P_FREQ1000:
              freq=1000;
              break;
            case P_FREQ500:
              freq=500;
              break;
            case P_FREQ100:
              freq=100;
              break;
          }           
          long limit = ((double)time/1000)/((double)_pauna_delay[_pauna_motorid]/(double)freq);
          if (mov == P_BACKWARD){
            limit = limit*-1;
          }
          rotateStep(limit);
          return true;
     }
  }
  return false;
}




bool PaunaStepper::move(PAUNA_STATES mov){
  switch (mov) {
    case P_STOP:
    case P_BRAKE:
    case P_FORWARD:
    case P_BACKWARD:
    case P_REVERSE:  
      pauna_setLimit(_pauna_motorid, 0);
      pauna_move(_pauna_motorid, mov);
      return true;
    break;
  }
  return false;
}

bool PaunaStepper::setStepDelayX(int vspeed){  
  if (vspeed > 0) {
    _pauna_delay[_pauna_motorid] = vspeed;
    return true;
  }
  return false;
}

bool PaunaStepper::setRPM(float rpm){  
  int freq;
  double res;
  if (rpm > 0) {
    switch (_pauna_freq) {
      case P_FREQ10000:
        freq=10000;
        break;
      case P_FREQ1000:
        freq=1000;
        break;
      case P_FREQ500:
        freq=500;
        break;
      case P_FREQ100:
        freq=100;
        break;
    }           
    _pauna_delay[_pauna_motorid] = ((60.0/rpm)/(float)_steps_per_revol)*(float)freq; 
    return true;
  }
  return false;
}

bool PaunaStepper::isMoving(){
  bool res;
  cli();
  res = _pauna_state[_pauna_motorid]!=P_STOPPED && _pauna_state[_pauna_motorid]!=P_DEATTACHED && _pauna_state[_pauna_motorid]!=P_BRAKED;
  sei();
  return res;
}

String PaunaStepper::getStateStr(){
  switch (_pauna_state[_pauna_motorid]){ 
    case  P_FORWARD:
      return "P_MFOWARD";
      break; 
    case  P_BACKWARD: 
      return "P_BACKWARD";
      break; 
    case  P_REVERSE:
      return "P_REVERSE";
      break; 
    case  P_BRAKE:
      return "P_BRAKE";
      break; 
    case  P_DEATTACHED:
      return "P_DEATTACHED";
      break; 
    case  P_STOP:
      break; 
      return "P_STOP";
    case  P_STOPPED:
      return "P_STOPPED";
      break; 
    case  P_BRAKED:
      return "P_BRAKED";
      break; 
  }
}

int PaunaStepper::getMotorId(){
  return _pauna_motorid;
}

