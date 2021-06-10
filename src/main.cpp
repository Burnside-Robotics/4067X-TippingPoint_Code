/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Alex Cutforth                                             */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Robot code for 4067X in the 2021-2022 Vex robotics        */
/*                  compeition | Tipping - Point                              */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

competition Competition;

motor lDrive1(PORT19, ratio18_1);
motor lDrive2(PORT20, ratio18_1);
motor rDrive1(PORT12, ratio18_1, true);
motor rDrive2(PORT11, ratio18_1, true);
motor_group lDrive(lDrive1, lDrive2);
motor_group rDrive(rDrive1, rDrive2);

motor lLift(PORT15, ratio36_1);
motor rLift(PORT16, ratio36_1, true);
motor_group lift(lLift, rLift);

controller Controller1;

const float MOTOR_ACCEL_LIMIT = 20;

const float USER_DRIVE_SPEED = 100;
const float AUTON_DRIVE_SPEED = 45;
const float AUTON_ROTATE_SPEED = 100;

const float ARM_SPEED = 100;

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

void autonomous(void) {
  
}

void usercontrol(void) {
  while (1) {
    setSideSpeeds(Controller1.Axis3.position() * USER_DRIVE_SPEED / 100,
                          Controller1.Axis2.position() * USER_DRIVE_SPEED / 100);

    if(Controller1.ButtonL1.pressing())
      lift.spin(fwd, ARM_SPEED, pct);
    else if(Controller1.ButtonL2.pressing())
      lift.spin(fwd, -ARM_SPEED, pct);
    else 
      lift.stop(hold);
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
