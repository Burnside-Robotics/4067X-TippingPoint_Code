#include "main.h"
#include "ARMS/api.h"

using namespace okapi;

Controller controller(ControllerId::master);

pros::ADIDigitalOut clamp1 = pros::ADIDigitalOut(1);
pros::ADIDigitalOut clamp2 = pros::ADIDigitalOut(8);

Motor lDrive1 = Motor(11);
Motor lDrive2 = Motor(14);
Motor lDrive3 = Motor(-15);

Motor rDrive1 = Motor(-1);
Motor rDrive2 = Motor(-4);
Motor rDrive3 = Motor(5);

MotorGroup lDrive = MotorGroup({ lDrive1, lDrive2, lDrive3 });
MotorGroup rDrive = MotorGroup({ rDrive1, rDrive2, rDrive3 });

Motor lLift = Motor(19);
Motor rLift = Motor(-9);
MotorGroup lift = MotorGroup({ lLift, rLift });

std::shared_ptr<ChassisController> driveTrain = 
	ChassisControllerBuilder()
		.withMotors(lDrive, rDrive)
		.withDimensions({ AbstractMotor::gearset::green }, { { 4_in, 13.7_in }, imev5GreenTPR })
		.withOdometry()
		.withGains(
			{ 0.25, 0, 00 }, // Distance controller
			{ 0.4, 0.05, 0 }, // Turn controller
			{ 0.001, 0, 0.001 } // Angle controller
		)
		.buildOdometry();;

std::shared_ptr<AsyncPositionController<double, double>> liftController = 
	AsyncPosControllerBuilder()
	.withMotor(lift)
	.build();

const double DRIVER_SPEED = 1;
const double AUTON_SPEED = 0.45;
const double AUTON_ROTATE_SPEED = 1;

const double LIFT_SPEED = 100;

const double MOTOR_ACCELERATION_LIMIT = 0.08;

int previousLeftSpeed = 0;
int previousRightSpeed = 0;

double clampSpeed(double& speed, double previousSpeed) {
	speed *= DRIVER_SPEED;
	if((speed - previousSpeed) > MOTOR_ACCELERATION_LIMIT) {
		speed = previousSpeed + MOTOR_ACCELERATION_LIMIT;
	}
	if((speed - previousSpeed) < -MOTOR_ACCELERATION_LIMIT) {
		speed = previousSpeed - MOTOR_ACCELERATION_LIMIT;
	}
}

void drive(double leftInput, double rightInput) {
	clampSpeed(leftInput, previousLeftSpeed);	
	clampSpeed(rightInput, previousRightSpeed);

	driveTrain -> getModel() -> tank(leftInput, rightInput);
}

void initialize() {
	pros::lcd::initialize();
	lift.setBrakeMode(AbstractMotor::brakeMode::hold);
	setClampOpen(true);
}

void competition_initialize() {}

void setClampOpen(bool open) {
	clamp1.set_value(open ? 1 : 0);
	clamp2.set_value(open ? 1 : 0);
}

void AutonSkills() {
	driveTrain -> moveDistance(113_cm);
	setClampOpen(false);
	driveTrain -> moveDistance(-85_cm);
 	setClampOpen(true);
 	driveTrain -> moveDistance(-20_cm);

 	driveTrain -> turnAngle(49_deg);
 	driveTrain -> moveDistance(35_cm);

	liftController -> setTarget(400);
	liftController -> waitUntilSettled();

	liftController -> setTarget(0);
	liftController -> waitUntilSettled();

	driveTrain -> turnAngle(-15_deg);
}

void autonomous() {
	switch(selector::auton) {
	case(0):
		// Skills
		break;
	}
}

bool aPressed = false;

bool clampOpen = false;

void opcontrol() {
	while(true) {
		drive(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));

		if(controller.getDigital(ControllerDigital::L1)) { 
			lift.moveVelocity(-LIFT_SPEED);
		} else if(controller.getDigital(ControllerDigital::L2)) {
			lift.moveVelocity(LIFT_SPEED);
		} else {
			lift.moveVelocity(0);
		}

		if(controller.getDigital(ControllerDigital::A) && !aPressed) {
			clampOpen = !clampOpen;
			aPressed = true;
		} else {
			aPressed = false;
		}

		setClampOpen(clampOpen);

		pros::delay(10);
	}
}
