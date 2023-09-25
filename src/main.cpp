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

brain Brain;
controller Controller1 = controller(primary);
motor MotorLF = motor(PORT20, ratio18_1, true); // reversed 
motor MotorLB = motor(PORT19, ratio18_1, true); // reversed
motor MotorRF = motor(PORT11, ratio18_1, true); // forward direction
motor MotorRB = motor(PORT12, ratio18_1, true); // forward direction
motor Shooter = motor(PORT5, ratio18_1, false);
motor LeftWing = motor(PORT6, ratio18_1, false);
motor RightWing = motor(PORT7, ratio18_1, false);
motor LIntake = motor(PORT1, ratio18_1, false);
motor RIntake = motor(PORT2, ratio18_1, false);
inertial DaInertial = inertial(PORT10);
motor_group LeftMotors = motor_group(MotorLF, MotorLB);
motor_group RightMotors = motor_group(MotorRF, MotorRB);


// A global instance of competition
competition Competition;


// Variables

bool ShootButtonPressed = false;
bool WingButtonPressed = false;

// define your global instances of motors and other devices here

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

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void accurateTurnCW(int deg){
    LeftMotors.setVelocity(50,pct);
    RightMotors.setVelocity(50,pct);
    //LeftMotors.spin(forward);
    //RightMotors.spin(forward);
    //MotorLF.spin(forward);
    //MotorLB.spin(forward);
    //MotorRF.spin(reverse);
    //MotorRB.spin(reverse);  
    waitUntil((DaInertial.rotation(degrees) >= deg));
    //LeftMotors.stop();
    //RightMotors.stop(); 
    //MotorLB.stop();
    //MotorLF.stop();
    //MotorRB.stop();
    //MotorRF.stop();

}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  accurateTurnCW(90);
}



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
    RightMotors.setVelocity((Controller1.Axis1.position() - Controller1.Axis3.position()), percent);
    LeftMotors.setVelocity((Controller1.Axis1.position() + Controller1.Axis3.position()), percent);
    RightMotors.spin(forward);
    LeftMotors.spin(forward);

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
