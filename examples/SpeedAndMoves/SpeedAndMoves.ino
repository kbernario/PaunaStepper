/*
 *    SpeedAndMoves.cpp - Rotate Stepper with Serial commands.
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

PaunaStepper stepper[2];   // array of stepper
int idx;                   // index of stepper to use

void setup() {
  Serial.begin(9600);
  // use: attach(pinToA, pinToB, pinToC, pinToD, step_delay, step_per_revolution, type_of_step)
  // Arduino "pin" to stepper controler pin A,B,C,D.
  stepper[0].attach(2, 3, 4, 5, 40, 4096, P_FULLSTEP);
  stepper[1].attach(8, 9, 10, 11, 60, 4096, P_FULLSTEP); 
  idx = 0;
}

void loop() {
  int option=1, aux;                              
  /* print menu */
  Serial.print("Options [ stepper selected: ");
  Serial.print(idx);
  Serial.println("]:");
  Serial.println("1) P_FORWARD");  
  Serial.println("2) P_BACKWARD");  
  Serial.println("3) P_BRAKE");  
  Serial.println("4) Decrease delay per step");  
  Serial.println("5) Increase delay per step");  
  Serial.println("6) Swap between stepper[0] or stepper[1]");  
  Serial.println("7) Print configuration");  
  Serial.println("waiting...\n\n");

  // while option is invalid wait for other command 
  while (option < '0' || option > '7') {
    if (Serial.available()) {  
      option = Serial.read();
      switch (option) {
        case '1': 
          stepper[idx].move(P_FORWARD);   // move forward
          break;
        case '2':
          stepper[idx].move(P_BACKWARD);  // move backward
          break;
        case '3':
          stepper[idx].move(P_BRAKE);     // stop and brake
          break;
        case '4':
          aux = stepper[idx].getStepDelayX()-1;  
          Serial.print("Set delayX to ");
          Serial.println(aux);
          stepper[idx].setStepDelayX(aux); // set step delay minus 1
          break;
        case '5':
          aux = stepper[idx].getStepDelayX()+1;  
          Serial.print("Set delayX to ");
          Serial.println(aux);
          stepper[idx].setStepDelayX(aux); // set step delay plus 1
          break;
        case '6':
          idx = !idx;                      // swap index of stepper to use
          Serial.print("Now use stepper ");
          Serial.println(idx);      
          break;
        case '7':
          Serial.print("Use stepper        : ");
          Serial.println(idx);      
          Serial.print("Odometry (in steps): ");
          Serial.println(stepper[idx].getOdo());         // return odometry (accomulative countstep)
          Serial.print("State of stepper   : ");
          Serial.println(stepper[idx].getStateStr());    // return state of stepper in string format
          Serial.print("IsMoving flag      : ");
          Serial.println(stepper[idx].isMoving());       // return true if is movig, false if is breaked, stopped o deattached      
          Serial.print("Time between steps (10 == 1 millisec) :");
          Serial.println(stepper[idx].getStepDelayX());  // return time between steps
          break;
      }  
    }
  }
  Serial.println("\n");
}

