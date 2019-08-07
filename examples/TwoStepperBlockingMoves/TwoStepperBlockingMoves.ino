/*
 *    TwoStepperBlocking.cpp - Rotate Stepper with blocking functions.
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
#include <PaunaStepper.h>

PaunaStepper stepper1;
PaunaStepper stepper2;

void setup() {
  // use: attach(pinToA, pinToB, pinToC, pinToD)
  // Arduino "pin" to stepper controler pin A,B,C,D.  
  stepper1.attach(2, 3, 4, 5);   // stepper with default parameters: delayX = 40, step per revolutions = 4096, type = P_HALFSTEP
  stepper2.attach(8, 9, 10, 11); // stepper with default parameters: delayX = 40, step per revolutions = 4096, type = P_HALFSTEP   
}

void loop() {
  // move stepper1 6 degree per second (90 degrees) (abosolute position).
  for (int i=0; i<90; i+=6) { 
    stepper1.waitRotateTo(i); 
    delay(1000);
  }
  
  // move stepper2 90 degree per iteration (360 degrees). (absolute position)
  for (int i=90; i<=360; i+=90) { 
    stepper2.waitRotateTo(i); 
    delay(1000);
  }

}
