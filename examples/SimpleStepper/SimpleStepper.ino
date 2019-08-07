/*
 *    SimpleStepper.cpp - Rotate Stepper with non blocking functions.
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

PaunaStepper stepper;

void setup() {
  Serial.begin(9600);
  // use: attach(pinToA, pinToB, pinToC, pinToD)
  // Arduino "pin" to stepper controler pin A,B,C,D.
  stepper.attach(2, 3, 4, 5);   // stepper with default parameters:  delay per step = 40, step per revolutions = 4096, type = P_HALFSTEP
}

void loop() {
  Serial.println("Loop");
  Serial.println("Move stepper Stepper to 180 degree (forward)");
  stepper.rotateTo(180);        //for relative position use stepper.rotate(180);
  Serial.println("\tPress [ENTER] to cancel move...");
  while (!Serial.available());  //wait for serial data
  Serial.read();
  
  stepper.rotateTo(0);          //for relative position use stepper.rotate(-180);
  Serial.println("\Move stepper Stepper to 0 degree (backward)");
  Serial.println("\tPress [ENTER] to cancel move and loop again...");
  while (!Serial.available());  //wait for serial data
  Serial.read();
  Serial.println("End Loop\n\n");
}

