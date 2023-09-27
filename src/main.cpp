/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       OLEG DA GREAT                                             */
/*    Created:      9/19/2023, 6:22:50 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here


brain Brain;
controller Controller1 = controller(primary);
motor MotorLF = motor(PORT20, ratio18_1, true); // reversed 
motor MotorLB = motor(PORT19, ratio18_1, true); // reversed
motor MotorRF = motor(PORT11, ratio18_1, false); // forward direction
motor MotorRB = motor(PORT12, ratio18_1, false); // forward direction
motor Shooter = motor(PORT5, ratio18_1, false);
motor LeftWing = motor(PORT6, ratio18_1, false);
motor RightWing = motor(PORT7, ratio18_1, false);
motor LIntake = motor(PORT1, ratio18_1, false);
motor RIntake = motor(PORT2, ratio18_1, false);
inertial DaInertial = inertial(PORT10);
motor_group LeftMotors = motor_group(MotorLF, MotorLB);
motor_group RightMotors = motor_group(MotorRF, MotorRB);





// Variables

bool ShootButtonPressed = false;
bool WingButtonPressed = false;


void event_Catapult(void){
      if (!ShootButtonPressed) {
        Shooter.setVelocity(80.0, percent);
        Shooter.spin(reverse);
        ShootButtonPressed = true;
      }
      else {
        Shooter.stop();
        ShootButtonPressed = false;
      };
};

void event_Wings(void){
    if (!WingButtonPressed) {
    	RightWing.setVelocity(70.0, percent);
      LeftWing.setVelocity(60.0, percent);
      RightWing.spinFor(reverse, 175.0, degrees, true);
      LeftWing.spinFor(forward, 175.0, degrees, true);
      LeftWing.stop();
      RightWing.stop();
      WingButtonPressed = true;
    }
    else {
      RightWing.setVelocity(70.0, percent);
      LeftWing.setVelocity(70.0, percent);
      LeftWing.spinFor(reverse, 175.0, degrees, true);
      RightWing.spinFor(forward, 175.0, degrees, true);
      LeftWing.stop();
      RightWing.stop();
      WingButtonPressed = false;
    }
}

void event_Intake(void){
  LIntake.setVelocity(100.0, percent);
  RIntake.setVelocity(100.0, percent);
  LIntake.spin(forward);
  RIntake.spin(reverse);
  waitUntil((!Controller1.ButtonR1.pressing()));
  LIntake.stop();
  RIntake.stop();
}

void event_Outake(void){
  LIntake.setVelocity(100.0, percent);
  RIntake.setVelocity(100.0, percent);
  LIntake.spin(reverse);
  RIntake.spin(forward);
  waitUntil((!Controller1.ButtonR2.pressing()));
  LIntake.stop();
  RIntake.stop();
}


void go_forward(void){
  LeftMotors.spin(forward);
  RightMotors.spin(forward); 
}

void go_backward(void){
  LeftMotors.spin(reverse);
  RightMotors.spin(reverse); 
}

void go_left(void){
  LeftMotors.spin(reverse);
  RightMotors.spin(forward); 
}

void go_right(void){
  LeftMotors.spin(forward);
  RightMotors.spin(reverse); 
}

void stop(void){
  LeftMotors.stop();
  RightMotors.stop(); 
}



void TurnRight(double desired){
  float Kp = 0.0842;
  float error = 0;
  float velocity;
  DaInertial.setRotation(0, degrees);
  while (DaInertial.rotation(degrees) <= desired) {
    error = desired -  DaInertial.rotation(degrees);
    velocity = Kp*error;
    LeftMotors.spin(forward, velocity, voltageUnits::volt);
    RightMotors.spin(reverse, velocity, voltageUnits::volt);
    wait(20, msec);
  }
    LeftMotors.stop();
    RightMotors.stop();
}

void TurnLeft(double desired){
  float Kp = 0.0842;
  float error = 0;
  float velocity;
  DaInertial.setRotation(0, degrees);
  while (fabs(DaInertial.rotation(degrees)) <= desired) {
    error = desired -  fabs(DaInertial.rotation(degrees));
    velocity = Kp*error;
    LeftMotors.spin(reverse, velocity, voltageUnits::volt);
    RightMotors.spin(forward, velocity, voltageUnits::volt);
    wait(20, msec);
  }
    LeftMotors.stop();
    RightMotors.stop();
}

void MoveForward(int cm){
  // 1 revolution = 25.9207cm
  // 60/34 Gear Ratio = 1.667
  // One Rev is 360 degrees and 25.9207*(60/36) = 43.20cm
  // cm to degree ration is 360/43.20 = 8.334
  RightMotors.resetPosition();
  LeftMotors.resetPosition();
  float degrees = 0;
  float distanceToDrive = cm*8.334;

  while(degrees<=distanceToDrive)
  { 
    LeftMotors.spin(forward, 3, voltageUnits::volt);
    RightMotors.spin(forward, 3, voltageUnits::volt);
    degrees = (LeftMotors.position(deg) + RightMotors.position(deg))/2;
    Brain.Screen.setCursor(12,1);
    Brain.Screen.print(degrees);
  }
    stop();
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  DaInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial");
  while (DaInertial.isCalibrating())
  {
    /* code */
    wait(20, msec);
  };
  Brain.Screen.clearScreen();

}

int ShowMeInfo(){
  while (true) {
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("Inertial Heading");
    Brain.Screen.setCursor(5,1);
    Brain.Screen.print(DaInertial.heading(degrees));
    Brain.Screen.setCursor(6,1);
    Brain.Screen.print("Inertial Rotate");
    Brain.Screen.setCursor(7,1);
    Brain.Screen.print(DaInertial.rotation(degrees)); 
    Brain.Screen.setCursor(8,1);
    Brain.Screen.print(" Motors");
    Brain.Screen.setCursor(9,1);
    Brain.Screen.print(LeftMotors.position(rotationUnits::deg)); 
    Brain.Screen.setCursor(10,1);
    Brain.Screen.print(RightMotors.position(rotationUnits::deg)); 

    wait(25, msec);
  } 
  return 0;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  vex::task MyTask(ShowMeInfo);
  //TurnRight(90);
  //wait(1,sec);
  //TurnLeft(90);
  MoveForward(50);
  }





/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  Controller1.ButtonL1.pressed(event_Catapult);
  Controller1.ButtonL2.pressed(event_Wings);
  Controller1.ButtonR2.pressed(event_Outake);
  Controller1.ButtonR1.pressed(event_Intake);
  wait(15, msec);
  // User control code here, inside the loop
  while (1) {
    RightMotors.setVelocity((Controller1.Axis3.position() - Controller1.Axis1.position()), percent);
    LeftMotors.setVelocity((Controller1.Axis1.position() + Controller1.Axis3.position()), percent);
    go_forward();
    

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);



  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
