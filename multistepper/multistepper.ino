// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.

#include <AccelStepper.h>
#include <MultiStepper.h>

// X direction
#define x_step 2
#define x_dir 23
#define limit_x 18

// Y direction
#define y_step 15
#define y_dir 22
#define limit_y 10

// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(
AccelStepper stepper1(AccelStepper::DRIVER, x_step, x_dir);
AccelStepper stepper2(AccelStepper::DRIVER, y_step, y_dir);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

// Array of desired stepper positions
long positions[2]; 
  
void setup() {
  Serial.begin(115200);

  pinMode(limit_x,INPUT);
  pinMode(limit_y,INPUT);
  

  // Configure each stepper
  stepper1.setMaxSpeed(10000);
  stepper2.setMaxSpeed(10000);
  stepper1.setAcceleration(5000);
  stepper2.setAcceleration(5000);

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

  // // move them untill each motor hits the limit switch
  // long currentPosition_x = stepper1.currentPosition();
  // long currentPosition_y = stepper2.currentPosition();

  // while(digitalRead(limit_x)){
  //   stepper1.move(1);
  // }

  // while(digitalRead(limit_y)){
  //   stepper2.move(1);
  // }

  // Serial.println("calibration completed");


  
  positions[0] = 100;
  positions[1] = 100;

}

void loop() {
  Serial.println("enter xpose,ypose:");
  if (Serial.available()) { // if there is data comming
  
    String command = Serial.readStringUntil('\n'); // read string until newline character
    
    String xpose = command.substring(0,command.indexOf(','));
    String ypose = command.substring(command.indexOf(',')+1);

    
    Serial.println("X:" + xpose);
    Serial.println("Y:" + ypose);

    positions[0] = xpose.toInt()*1000;
    positions[1] = ypose.toInt()*1000;
    
  }



  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
  
  
}
