/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       OLEG KHAVRONIUK                                           */
/*    Created:      9/19/2023, 6:22:50 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "PID.h"

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
motor Shooter = motor(PORT4, ratio36_1, false);
motor LeftWing = motor(PORT6, ratio18_1, false);
motor RightWing = motor(PORT7, ratio18_1, false);
motor LIntake = motor(PORT1, ratio18_1, false);
motor RIntake = motor(PORT2, ratio18_1, false);
motor Arm = motor(PORT8, ratio36_1, false);
inertial DaInertial = inertial(PORT10);
motor_group LeftMotors = motor_group(MotorLF, MotorLB);
motor_group RightMotors = motor_group(MotorRF, MotorRB);

// Global Variables & Constants
bool ShootButtonPressed = false;
bool WingButtonPressed = false;
bool ArmButtonPressed = false;
bool RWingButtonPressed = false;
PID drivePID;
PID turnPID;
PID straightPID;
  // 1 revolution = ~26cm
  // 60/36 Gear Ratio
  // One Rev is 360 degrees and 25.9207*(60/36) = 43.20cm
  // cm to degree ration is 360/43.20 = 8.334
float const WHEEL_DIAMETER = 8.3; // cm
float const WHEEL_CIRC = WHEEL_DIAMETER * 3.14;
float const GEAR_RATIO = 1.67;
float const DEGREE_PER_CM = 360 / (WHEEL_CIRC * GEAR_RATIO);

bool isArmOpen(){
  int encoderValue = Arm.position(deg);
  if(encoderValue > 40){
    return true;
  }
  return false;
}

bool isRightWOpen(){
  int encoderValue = RightWing.position(deg);
  if(encoderValue < -40){
    return true;
  }
  return false;
}

bool isLeftWOpen(){
  int encoderValue = LeftWing.position(deg);
  if(encoderValue > 40){
    return true;
  }
  return false;
}


void push(){
 LeftMotors.setVelocity(60,pct);
 RightMotors.setVelocity(60,pct);
 LeftMotors.spin(forward);
 RightMotors.spin(forward);
 wait(300,msec);
 LeftMotors.stop();
 RightMotors.stop();
}


void event_Wings(void){
    if (!isArmOpen() && !WingButtonPressed) {
      if(!isRightWOpen())
      {
        RightWing.setVelocity(70.0, percent);
        //RightWing.spinFor(reverse, 175.0, degrees, true);
        RightWing.spinTo(-140, deg,true);
        RightWing.stop();
      }
      if(!isLeftWOpen()){
        LeftWing.setVelocity(60.0, percent);      
        LeftWing.spinTo(140, deg,true);
        //LeftWing.spinFor(forward, 175.0, degrees, true);
        LeftWing.stop(); 
      }
      WingButtonPressed = true;
    }
    else if (!isArmOpen() && WingButtonPressed) {
      if(isLeftWOpen()){
        LeftWing.setVelocity(60.0, percent);      
        //LeftWing.spinFor(reverse, 175.0, degrees, true);
        LeftWing.spinTo(20, deg,true);
        LeftWing.stop(); 
      }
      if(isRightWOpen())
      {
        RightWing.setVelocity(70.0, percent);
         RightWing.spinTo(-20, deg,true);
        //RightWing.spinFor(forward, 175.0, degrees, true);
        RightWing.stop();
      }
      WingButtonPressed = false;
    }
}

void event_RightWing(void){
  if (!isArmOpen() && !RWingButtonPressed)
 {
    if(!isRightWOpen())
    {
      RightWing.setVelocity(70.0, percent);
      RightWing.spinTo(-140, deg,true);
      RightWing.stop();
    }
    RWingButtonPressed = true;
 }
 else if (!isArmOpen() && RWingButtonPressed) {
    if(isRightWOpen())
    {
      RightWing.setVelocity(70.0, percent);
      RightWing.spinTo(-20, deg,true);
      RightWing.stop();
    }
    RWingButtonPressed = false;
     }
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

void event_Arm(void){
    if (!isArmOpen()) {
      Arm.setStopping(coast);
    	Arm.setVelocity(70.0, percent);
      Arm.spinTo(75,deg, true); //was 170
      Arm.stop(hold);
    }
    else {
      Arm.setVelocity(70.0, percent);
      Arm.setStopping(coast);
      Arm.spinTo(30,deg,true);
      Arm.stop();
    }
}


void turn_right(int DegreesToTurn, float VelocityMin=2, float VelocityMax=12) {
  float speed;
  float currentDegrees;
  setPIDmin(turnPID, VelocityMin);
  setPIDmax(turnPID, VelocityMax);
  DaInertial.resetRotation();
  do {
    wait(20, msec);
    currentDegrees = DaInertial.rotation();
    speed = calculatePID(turnPID,DegreesToTurn, currentDegrees);
    RightMotors.spin(reverse, speed, volt);
    LeftMotors.spin(forward, speed, volt);
  } while(DegreesToTurn - currentDegrees > 0);
  RightMotors.stop(hold);
  LeftMotors.stop(hold);
  resetPID(turnPID);
}

void turn_left(int DegreesToTurn, float VelocityMin=2, float VelocityMax=12) {
  float speed;
  float currentDegrees;
  DaInertial.resetRotation();
  setPIDmin(turnPID, VelocityMin);
  setPIDmax(turnPID, VelocityMax);
  do {
    wait(20, msec);
    currentDegrees = fabs(DaInertial.rotation());
    speed = calculatePID(turnPID, DegreesToTurn, currentDegrees);
    RightMotors.spin(forward, speed, volt);
    LeftMotors.spin(reverse, speed, volt);
  } while(DegreesToTurn - currentDegrees > 0);
  RightMotors.stop(hold);
  LeftMotors.stop(hold);
  resetPID(turnPID);
}

void drive_forward(int distanceToDrive, float VelocityMin=2, float VelocityMax=12){
  float yaw;
  double correction;
  float currentDegree;
  double speed;
  float degreeToDrive = DEGREE_PER_CM * distanceToDrive;
  RightMotors.resetPosition();
  LeftMotors.resetPosition();
  DaInertial.resetRotation();
  setPIDmax(drivePID, VelocityMax);
  setPIDmin(drivePID, VelocityMin);
  do {
    wait(20,msec);
    currentDegree = (RightMotors.position(deg) + LeftMotors.position(deg)) / 2;
    speed = calculatePID(drivePID, degreeToDrive, currentDegree);
    yaw = DaInertial.rotation();
    correction = calculatePID(straightPID, 0, yaw);

    RightMotors.spin(forward, speed - correction, volt);
    LeftMotors.spin(forward, speed + correction, volt);
  } while(degreeToDrive - currentDegree > 3);
  RightMotors.stop(brake);
  LeftMotors.stop(brake);
  resetPID(drivePID);
  resetPID(straightPID);
}

void drive_backward(int distanceToDrive, float VelocityMin=2, float VelocityMax=12){
  float yaw;
  double correction;
  float currentDegree;
  float speed;
  float degreeToDrive = DEGREE_PER_CM * distanceToDrive;
  RightMotors.resetPosition();
  LeftMotors.resetPosition();
  DaInertial.resetRotation();
  setPIDmax(drivePID, VelocityMax);
  setPIDmin(drivePID, VelocityMin);
  do {
    wait(20,msec);
    currentDegree = (fabs(RightMotors.position(deg)) + fabs(LeftMotors.position(deg))) / 2;
    speed = calculatePID(drivePID, degreeToDrive, currentDegree);
    yaw = DaInertial.rotation();
    correction = calculatePID(straightPID, 0, yaw);
    LeftMotors.spin(reverse, speed - correction, volt);
    RightMotors.spin(reverse, speed + correction, volt);
  } while(degreeToDrive - currentDegree > 3);

  LeftMotors.stop(brake);
  RightMotors.stop(brake);
  resetPID(drivePID);
  resetPID(straightPID);
}

void outake_off(void){
  LIntake.stop();
  RIntake.stop();
}

void outake_on(void){
  LIntake.setVelocity(100, pct);
  RIntake.setVelocity(100, pct);
  LIntake.spin(reverse);
  RIntake.spin(forward);
}

void intake_on(void){
  LIntake.setVelocity(100, pct);
  RIntake.setVelocity(100, pct);
  LIntake.spin(forward);
  RIntake.spin(reverse);
}

void Arm_Move(void){
  Arm.setVelocity(80, pct);
  Arm.setMaxTorque(100, pct);
  Arm.setBrake(coast);
  Arm.spinToPosition(175, deg, true);
  Arm.stop();
}

void Arm_Move_back(void){
  Arm.setVelocity(80, pct);
  Arm.setMaxTorque(100, pct);
  Arm.setBrake(coast);
  Arm.spinToPosition(20, deg, true);
  Arm.stop();
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
  Brain.Screen.print("Calibrating Inertial Sensor");
  while (DaInertial.isCalibrating())
  {
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
    Brain.Screen.print("Inertial Rotation");
    Brain.Screen.setCursor(7,1);
    Brain.Screen.print(DaInertial.rotation(degrees)); 
    Brain.Screen.setCursor(8,1);
    Brain.Screen.print(" Motors Left/Right/Arm");
    Brain.Screen.setCursor(9,1);
    Brain.Screen.print(LeftMotors.position(rotationUnits::deg)); 
    Brain.Screen.setCursor(10,1);
    Brain.Screen.print(RightMotors.position(rotationUnits::deg));
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print(Arm.position(rotationUnits::deg));
    wait(25, msec);
  } 
  return 0;
}

void auto_own(void){
  int speedMin = 2;
  int speedMax = 7;
  int turnSpeedMin = 2;
  int turnSpeedMax = 6;
  drive_backward(120, speedMin, speedMax);
  wait(20, msec);
  turn_right(90, turnSpeedMin, turnSpeedMax);
  wait(20, msec);
  drive_forward(7, 4, speedMax);
  wait(20, msec);
  outake_on();
  drive_forward(3, 4, speedMax);
  wait(300,msec);
  outake_off();
  wait(20,msec);
  drive_backward(10, 6, speedMax);
  wait(20,msec);
  turn_right(90, turnSpeedMin, speedMax);
  wait(20,msec);
  drive_backward(55, speedMin, speedMax);
  wait(20,msec);
  turn_right(45, turnSpeedMin, speedMax);
  wait(20,msec);
  drive_backward(66, speedMin, speedMax);
  wait(20,msec);
  Arm_Move();
  wait(20,msec);
  turn_left(125, turnSpeedMin, turnSpeedMax);
  wait(20,msec);
  Arm.spinToPosition(90, deg, false);
  wait(20,msec);
  // drive_backward(115, 4.5, speedMax);
  drive_backward(75, 4.5, speedMax);
  wait(20,msec);
  turn_left(5, 4,8);
  wait(20,msec);
  drive_backward(38, 4.5, speedMax); 

}

/*
void auto_own_alone(void){
  int speedMin = 2;
  int speedMax = 7;
  int turnSpeedMin = 2;
  int turnSpeedMax = 6;
  drive_backward(120, speedMin, speedMax);
  wait(20, msec);
  turn_right(90, turnSpeedMin, turnSpeedMax);
  wait(20, msec);
  outake_on();
  wait(20,msec);
  push();
  outake_off();
  wait(20,msec);
  drive_backward(67, speedMin, speedMax);
}
*/


void auto_opposite_prototype(void){
  int speedMin = 2;
  int speedMax = 7;
  int turnSpeedMin = 2;
  int turnSpeedMax = 6;
  drive_backward(120, 5, speedMax);
  wait(20, msec);
  turn_left(90, 3, turnSpeedMax);
  wait(20, msec);
  drive_forward(7, 4, speedMax);
  wait(20, msec);
  outake_on();
  drive_forward(3, 4, speedMax);
  wait(300,msec);
  outake_off();
  drive_backward(80, 4.5, speedMax);
  event_Wings();
  wait(20, msec);
  drive_forward(50, speedMin, speedMax);
  event_Wings();
  drive_backward(10, 4, speedMax);
  wait(20, msec);
  turn_right(45, turnSpeedMin, turnSpeedMax);
  wait(20, msec);
  drive_backward(40, speedMin, speedMax);
}


void auto_opposite(void){
  int speedMin = 2;
  int speedMax = 7;
  int turnSpeedMin = 2;
  int turnSpeedMax = 6;
  drive_backward(120, 4, speedMax);
  wait(20, msec);
  turn_left(90, 3, turnSpeedMax);
  wait(20, msec);
  drive_forward(7, 4, speedMax);
  wait(20, msec);
  outake_on();
  drive_forward(3, 4, speedMax);
  wait(300,msec);
  outake_off();
  drive_backward(40, 3, speedMax);
  wait(20, msec);
  event_Wings();
  turn_right(90, 3, turnSpeedMax);
  wait(20, msec);
  event_Wings();
  wait(20, msec);
  turn_right(90, 3, turnSpeedMax);
  wait(20, msec);
  event_Wings();
  drive_backward(45, 4.5, speedMax);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.*/
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  vex::task MyTask(ShowMeInfo);
  //auto_opposite();
  auto_own();
  //auto_own_alone();
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
  Controller1.ButtonB.pressed(event_Arm);
  Controller1.ButtonRight.pressed(event_RightWing);
  wait(15, msec);
  // User control code here, inside the loop
  while (1) {
    RightMotors.setVelocity((Controller1.Axis3.position() - Controller1.Axis1.position()), percent);
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
  initPID(drivePID, 0.02, 0.001, 0.01, 40, 2, 10);
  initPID(turnPID, 0.08, 0.009, 0.05, 5, 2, 10);
  initPID(straightPID, 0.15, 0, 0.4, 1, -3, 3);
  ShootButtonPressed = false;
  WingButtonPressed = false;
  LeftWing.resetPosition();
  RightWing.resetPosition();
  Arm.resetPosition();

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
