/*
 *    TwoStepperBlocking.cpp - Rotate Stepper with non blocking functions.
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
  Serial.begin(9600);
  // use: attach(pinToA, pinToB, pinToC, pinToD)
  // Arduino "pin" to stepper controler pin A,B,C,D.  
  stepper1.attach(2, 3, 4, 5);   // stepper with default parameters:  delay per step = 40, step per revolutions = 4096, type = P_HALFSTEP
  stepper2.attach(8, 9, 10, 11); // stepper with default parameters:  delay per step = 40, step per revolutions = 4096, type = P_HALFSTEP
}

void loop() {
  Serial.println("Two Steppers rotate 360 and -360 degrees(no blocking)");
  Serial.print("Set 6 RPM -> ");  
  stepper1.setRPM(6);
  stepper2.setRPM(6);
  Serial.print(stepper1.getStepDelayX());  
  Serial.println(" delay time per step");    
  stepper1.moveFor(P_FORWARD, 10000); // 10 secs = 1 revolution at 6 RPM
  stepper2.rotate(-360);
  Serial.println("Two Stepper STOP automatically. Waiting...");  
  while (stepper1.isMoving() || stepper2.isMoving()){
     delay(1000);
     Serial.print("\tstepper 1 (steps):");
     Serial.print(stepper1.getOdo());
     Serial.print(" stepper 2 (steps):");
     Serial.println(stepper2.getOdo());
  }
  Serial.println("\tPosition reached, reset odometry\n\n");
  stepper1.resetOdo();
  stepper2.resetOdo();
  delay(3000);
}

