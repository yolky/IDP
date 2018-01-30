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
	initialiseSensors();
}

void Robot::initialiseSensors() {
	leftSpeed = 0;
	rightSpeed = 0;
	for(int i = 0; i< 4; i++){
        lineSensors[i] = false;
	}
	watch.start();
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
	int uIntSpeed = doubleToUInt(-1.0 * limitSpeed(speed));
	if (uIntSpeed != leftSpeed) {
        leftSpeed = uIntSpeed;
		rlink.command(MOTOR_1_GO, uIntSpeed);
	}
}

double Robot::limitSpeed(double speed){
    double limitedSpeed = max(speed, -1.0);
    return min<double>(limitedSpeed, 1.0);
}

void Robot::setRightMotor(double speed){
	int uIntSpeed = doubleToUInt(limitSpeed(speed));
	if (uIntSpeed != rightSpeed) {
        rightSpeed = uIntSpeed;
		rlink.command(MOTOR_2_GO, uIntSpeed);
	}
}

void Robot::setMotors(double leftSpeed, double rightSpeed){
    //cout << leftSpeed << ',' << rightSpeed << endl;
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

void Robot::updateLineSensors() {
	int sensorValues = rlink.request(READ_PORT_0);
	int currentTime = watch.read();
	for (int i = 0; i < 4; i++) {
        int sensorValue = sensorValues %2;
		lineSensors[i] = sensorValue;
		lineSensorsLastTriggered[sensorValue][i] = currentTime;
		sensorValues = sensorValues >> 1;
	}
}

bool Robot::checkLineSensorsMatch(int sensorState[]) {
	for (int i = 0; i < 4; i++) {
		if (sensorState[i] != -1 && sensorState[i] != lineSensors[i]) {
			return false;
		}
	}
	return true;
}


bool Robot::checkLineSensorsMatchFuzzy(int sensorState[], int interval) {
    int currentTime = watch.read();
	for (int i = 0; i < 4; i++) {
		if (sensorState[i] != -1 && sensorState[i] != lineSensors[i] && currentTime-lineSensorsLastTriggered[sensorState[i]][i] > interval) {
			return false;
		}
	}
	return true;
}
