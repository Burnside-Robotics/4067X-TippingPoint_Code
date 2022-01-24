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
// pneumatic2         limit         B               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

competition Competition;

motor lDrive1(PORT19, ratio18_1);
motor lDrive2(PORT20, ratio18_1);
motor lDrive3(PORT18, ratio18_1, true);
motor rDrive1(PORT12, ratio18_1, true);
motor rDrive2(PORT11, ratio18_1, true);
motor rDrive3(PORT13, ratio18_1);
motor_group lDrive(lDrive1, lDrive2, lDrive3);
motor_group rDrive(rDrive1, rDrive2, rDrive3);

motor dragger(PORT8, ratio18_1);

motor lLift(PORT9, ratio36_1);
motor rLift(PORT10, ratio36_1, true);
motor_group lift(lLift, rLift);

controller Controller1;

const float MOTOR_ACCEL_LIMIT = 30;

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

void autonomous(void) {
  setClampOpen(true);
  DriveDistance(113);
  setClampOpen(false);
  DriveDistance(-85);
  setClampOpen(true);
  DriveDistance(-20);
  RotateDegrees(35);
  DriveDistance(40);
  setClampOpen(false);
  DriveDistance(-40);
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

    setClampOpen(clampOpen);
    wait(20, msec);
  }
}

void toggleClamp() {
  clampOpen = !clampOpen;
}


int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  Controller1.ButtonA.pressed(toggleClamp);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
