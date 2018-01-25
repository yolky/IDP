#include <iostream>
#include "Robot.h"
#include <robot_link.h>
#include <robot_delay.h>
#include <robot_instr.h>
#include <math.h>
#include <stdint.h>
#define ROBOT_NUM 16

Robot::Robot(){
	initialiseRobotLink();
}

void Robot::initialiseRobotLink(){
	#ifdef __arm__
		if (!rlink.initialise ("127.0.0.1")) {          // setup for local hardware
	#else
	   if (!rlink.initialise (ROBOT_NUM)) { // setup the link
	#endif
		cout << "Cannot initialise link" << endl;
		rlink.print_errs("    ");
	}
}

void Robot::setLeftMotor(double speed){
	int uIntSpeed = doubleToUInt(-1.0 * speed);
	cout<< uIntSpeed << endl;
	rlink.command(MOTOR_1_GO, uIntSpeed);
}

void Robot::setRightMotor(double speed){
	int uIntSpeed = doubleToUInt(speed);
		cout<< uIntSpeed << endl;

	rlink.command(MOTOR_2_GO, uIntSpeed);
}

void Robot::setMotors(double leftSpeed, double rightSpeed){
	setLeftMotor(leftSpeed);
	setRightMotor(rightSpeed);

}

int Robot::doubleToUInt(double number){
	int asInt = round(number * 127);

	
	return (asInt>0)? asInt : (-1 * asInt) + 128;
}

int Robot::sendTestInstruction(){
	return rlink.request (TEST_INSTRUCTION);
}

void Robot::printErrors(){
	rlink.print_errs();
}
