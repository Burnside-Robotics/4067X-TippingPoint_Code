/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Alex Cutforth                                             */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Robot code for 4067X in the 2021-2022 Vex robotics        */
/*                  compeition | Tipping - Point                              */
/*----------------------------------------------------------------------------*/

#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// pneumatic1         limit         A      
// pneumatic2         limit         H               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

competition Competition;

motor lDrive1(PORT11, ratio18_1);
motor lDrive2(PORT14, ratio18_1);
motor lDrive3(PORT15, ratio18_1, true);
motor rDrive1(PORT1, ratio18_1, true);
motor rDrive2(PORT4, ratio18_1, true);
motor rDrive3(PORT5, ratio18_1);
motor_group lDrive(lDrive1, lDrive2, lDrive3);
motor_group rDrive(rDrive1, rDrive2, rDrive3);

motor dragger(PORT8, ratio18_1);

motor lLift(PORT19, ratio36_1);
motor rLift(PORT9, ratio36_1, true);
motor_group lift(lLift, rLift);

controller Controller1;

int MOTOR_ACCEL_LIMIT = 8;

const float USER_DRIVE_SPEED = 100;
const float AUTON_DRIVE_SPEED = 45;
const float AUTON_ROTATE_SPEED = 100;

const float ARM_SPEED = 100;
const float CONVEYOR_SPEED = 60;

float wheelCircumfrence = 31.9185813596f;
float turningDiameter = 52;

int s_lastL = 0;
int s_lastR = 0;
void setSideSpeeds(int lSpeed, int rSpeed)
{
    if ((lSpeed - s_lastL) > MOTOR_ACCEL_LIMIT)
        lSpeed = s_lastL + MOTOR_ACCEL_LIMIT;
    if ((lSpeed - s_lastL) < -MOTOR_ACCEL_LIMIT)
        lSpeed = s_lastL - MOTOR_ACCEL_LIMIT;
    if ((rSpeed - s_lastR) > MOTOR_ACCEL_LIMIT)
        rSpeed = s_lastR + MOTOR_ACCEL_LIMIT;
    if ((rSpeed - s_lastR) < -MOTOR_ACCEL_LIMIT)
        rSpeed = s_lastR - MOTOR_ACCEL_LIMIT;

    s_lastL = lSpeed;
    s_lastR = rSpeed;

    if (lSpeed == 0)
        lDrive.stop(brakeType::brake);
    else
        lDrive.spin(directionType::fwd, lSpeed, velocityUnits::pct);
    if (rSpeed == 0)
        rDrive.stop(brakeType::brake);
    else
        rDrive.spin(directionType::fwd, rSpeed, velocityUnits::pct);
}

void pre_auton(void) {
  vexcodeInit();
}

void DriveDistance(float distance)
{
    lDrive1.resetPosition();
    rDrive1.resetPosition();

    float rotationGoal = (360 * distance) / wheelCircumfrence;

    float maxSpeed = 0;
    const float Kp = 0.25;
    const float Ki = 0;
    const float Kd = 0;
    const float deadZone = 1;

    float lError = 0;
    float rError = 0;
    float lIntegral = 0;
    float rIntegral = 0;

    float lDerivative = 0;
    float rDerivative = 0;

    float lLastError = 0;
    float rLastError = 0;

    float lMotorSpeed = 0;
    float rMotorSpeed = 0;

    float doneTime = 0;
    while (true)
    {
        lError = rotationGoal - lDrive1.rotation(deg);
        rError = rotationGoal - rDrive1.rotation(deg);

        lIntegral += lError;
        rIntegral += rError;
        if (lError > 200 || lError < -200)
        {
            lIntegral = 0;
        }
        if (rError > 200 || rError < -200)
        {
            rIntegral = 0;
        }

        lDerivative = lError - lLastError;
        rDerivative = rError - rLastError;

        lLastError = lError;
        rLastError = rError;

        lMotorSpeed = Kp * lError + Ki * lIntegral + Kd * lDerivative;
        rMotorSpeed = Kp * rError + Ki * rIntegral + Kd * rDerivative;
        if (doneTime < 500)
        {
            if (distance < 0)
                maxSpeed = -(doneTime / 5);
            else
                maxSpeed = doneTime / 5;
        }
        else
        {

            if (lMotorSpeed < deadZone && lMotorSpeed > -deadZone && rMotorSpeed < deadZone && rMotorSpeed > -deadZone)
            {
                rDrive.spin(fwd, 0, pct);
                lDrive.spin(fwd, 0, pct);
                break;
            }
        }
        if (distance > 0)
        {
            lMotorSpeed = maxSpeed < lMotorSpeed ? maxSpeed : lMotorSpeed;
            rMotorSpeed = maxSpeed < rMotorSpeed ? maxSpeed : rMotorSpeed;
        }
        else
        {
            lMotorSpeed = maxSpeed > lMotorSpeed ? maxSpeed : lMotorSpeed;
            rMotorSpeed = maxSpeed > rMotorSpeed ? maxSpeed : rMotorSpeed;
        }

        lDrive.spin(fwd, lMotorSpeed, pct);
        rDrive.spin(fwd, rMotorSpeed, pct);

        wait(15, msec);
        doneTime += 15;
    }
}
void RotateDegrees(int degrees)
{
    lDrive1.resetPosition();
    rDrive1.resetPosition();

    float maxSpeed = 100;

    const float Kp = 0.4;
    const float Ki = 0.05;
    const float Kd = 0;
    const float deadZone = 1;

    float error = 0;
    float integral = 0;

    float derivative = 0;

    float lastError = 0;

    float motorSpeed = 0;

    float doneTime = 0;
    
    float turningRatio = turningDiameter / (wheelCircumfrence / 3.1415926535);
    float target = turningRatio * degrees;
    while (true)
    {
        error = target - (lDrive1.rotation(deg) - rDrive1.rotation(deg)) / 2;
        integral += error;
        if (error > 40 || error < -40)
        {
            integral = 0;
            derivative = 0;
        }
        else
        {
            if (motorSpeed < deadZone && motorSpeed > -deadZone)
            {
                rDrive.spin(fwd, 0, pct);
                lDrive.spin(fwd, 0, pct);
                break;
            }
        }
        derivative = error - lastError;

        lastError = error;

        motorSpeed = Kp * error + Ki * integral + Kd * derivative;

        if (doneTime < 200)
        {
            if (degrees < 0)
                maxSpeed = -(doneTime / 2);
            else
                maxSpeed = doneTime / 2;
        }

        if (degrees > 0)
        {
            motorSpeed = maxSpeed < motorSpeed ? maxSpeed : motorSpeed;
        }
        else
        {
            motorSpeed = maxSpeed > motorSpeed ? maxSpeed : motorSpeed;
        }
        lDrive.spin(fwd, motorSpeed, pct);
        rDrive.spin(fwd, -motorSpeed, pct);
        if (int(doneTime) % 60 == 0)
        {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(4, 4);
            Brain.Screen.print(target);
            Brain.Screen.setCursor(6, 6);
            Brain.Screen.print(error);
        }
        wait(15, msec);
        doneTime += 15;
    }
}

void setClampOpen(bool val) {
  clamp1.set(val);
  clamp2.set(val);
}

void rotateLift(int degrees, bool async) {
  directionType dir = degrees > 0 ? directionType::rev : directionType::fwd;
  lLift.startRotateFor(dir, abs(degrees), deg);
  if(async) {
    rLift.startRotateFor(dir, abs(degrees), deg);
  } else {
    rLift.rotateFor(dir, abs(degrees), deg);
  }
}

int auton = 0;

void AutonRight() {
  setClampOpen(true);
  DriveDistance(113);
  setClampOpen(false);
  DriveDistance(-85);
  setClampOpen(true);
  DriveDistance(-20);
  RotateDegrees(38);
  DriveDistance(43);
  setClampOpen(false);
  DriveDistance(-30);
  setClampOpen(true);
  DriveDistance(-10);
  RotateDegrees(15);
  rotateLift(400, false);
  rotateLift(-350, false);
}

void AutonLeft() {
  setClampOpen(true);
  DriveDistance(120);
  setClampOpen(false);
  DriveDistance(-85);
  setClampOpen(true);
  DriveDistance(-35);
  RotateDegrees(90);
  rotateLift(400, false);
  rotateLift(-350, false);
}


void AutonSkills() {
  setClampOpen(true);
  DriveDistance(113);
  setClampOpen(false);
  DriveDistance(-85);
  setClampOpen(true);
  DriveDistance(-20);
  RotateDegrees(49);
  DriveDistance(35);
  rotateLift(400, false);
  rotateLift(-400, false);
  RotateDegrees(-15);
  DriveDistance(10);
  setClampOpen(false);
  RotateDegrees(-35);
  DriveDistance(240);
  setClampOpen(true);
}

void autonomous(void) {
  if(auton == 0) {
    AutonRight();
  } else if(auton == 1) {
    AutonLeft();
  } else {
    AutonSkills();
  }
}

bool clampOpen = true;

void usercontrol(void) {
  while (1) {
    setSideSpeeds(Controller1.Axis3.position() * USER_DRIVE_SPEED / 100,
                          Controller1.Axis2.position() * USER_DRIVE_SPEED / 100);
    if(Controller1.ButtonL1.pressing())
      lift.spin(fwd, -ARM_SPEED, pct);
    else if(Controller1.ButtonL2.pressing())
      lift.spin(fwd, ARM_SPEED, pct);
    else 
      lift.stop(hold);

    if(Controller1.ButtonR1.pressing())
      dragger.spin(fwd, -20, pct);
    else if(Controller1.ButtonR2.pressing())
      dragger.spin(fwd, 20, pct);
    else 
      dragger.stop(hold);

    // handleControllerDisplay();
    setClampOpen(clampOpen);
    wait(20, msec);
  }
}

void toggleClamp() {
  clampOpen = !clampOpen;
}

void increaseSensitivity() {
  MOTOR_ACCEL_LIMIT++;
}

void decreaseSensitivity() {
  if(MOTOR_ACCEL_LIMIT > 5) {
    MOTOR_ACCEL_LIMIT--;
  }
}

void toggleAuton() {
  auton++;
  if(auton == 3) {
    auton = 0;
  }
  Controller1.Screen.clearLine();
  Controller1.Screen.print("Auton: ");
  if(auton == 0) {
    Controller1.Screen.print("Right");
  } else if(auton == 1) {
    Controller1.Screen.print("Left");
  } else {
    Controller1.Screen.print("Skills");
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  Controller1.ButtonA.pressed(toggleClamp);
  Controller1.ButtonUp.pressed(increaseSensitivity); 
  Controller1.ButtonDown.pressed(decreaseSensitivity);
  Controller1.ButtonX.pressed(toggleAuton);
  pre_auton();
  while (true) {
    wait(100, msec);
    // handleControllerDisplay();
  }
}
