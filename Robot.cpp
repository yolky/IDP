#include <iostream>
#include "Robot.h"
#include <robot_link.h>
#include <robot_delay.h>
#include <robot_instr.h>
#include <math.h>
#include <stdint.h>
#define ROBOT_NUM 16
#include <fstream>
#include <iostream>

Robot::Robot(){
	initialiseRobotLink();
	initialiseSensors();
	myfile.open("test.txt");
    myfile2.open("test2.txt");
}

void Robot::initialiseSensors() {
	leftSpeed = 0;
	rightSpeed = 0;
	for(int i = 0; i< 4; i++){
        lineSensors[i] = false;
        for(int g =0; g<2; g++){
            lineSensorsLastTriggered[g][i] = -10000;
        }
	}

	for(int i =0; i< NUMBER_VALUES_STORED; i++){
        previousDistanceValues[i] = -1;
	}

	for(int i =0; i< DIFFERENTIATION_INTERVAL_HARVESTING; i++){
        previousDistanceValues[i] = -1;
	}

	for(int i = 0; i< 8; i++){
        pinValues[i] = 0;
	}
	previousDistanceIndex = 0;
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

void Robot::setMechanismMotor(double speed){
	int uIntSpeed = doubleToUInt(-1.0 * limitSpeed(speed));
	if (uIntSpeed != leftSpeed) {
        leftSpeed = uIntSpeed;
		rlink.command(MOTOR_3_GO, uIntSpeed);
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

void Robot::stopMotors(){
    setLeftMotor(0);
    setRightMotor(0);
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
    readPinValue(0);
    int sensorValues = pinValues[0];
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

void Robot::activateHarvestMechanism(){
    int newPinValue = setBitValue(pinValues[1], 0, 1);
    if(newPinValue != pinValues[1]){
        pinValues[1] = newPinValue;
        rlink.command(WRITE_PORT_1, pinValues[1]);
    }
}

void Robot::deactivateHarvestMechanism(){
    int newPinValue = setBitValue(pinValues[1], 0, 0);
    if(newPinValue != pinValues[1]){
        pinValues[1] = newPinValue;
        rlink.command(WRITE_PORT_1, pinValues[1]);
    }
}

int Robot::setBitValue(int number, int bit, bool value){ //bit number starts at 0
    return value? number | (int)pow(2, bit) : number & (255 - (int)pow(2, bit));
}

void Robot::updateDistanceSensor(){
    distanceSensorVoltage = rlink.request(ADC3);
    previousDistanceValues[previousDistanceIndex] = distanceSensorVoltage;
    previousDistanceIndex  = (previousDistanceIndex + 1)%NUMBER_VALUES_STORED;
}

void Robot::updateLDRFront(bool updateMean){
    LDRFront = rlink.request(ADC4);
    //cout<< " mean " << meanLDR <<endl << "    sss    " << LDRFront << endl;;
    myfile << LDRFront << "\n";
    myfile2 << meanLDR << "\n";
    if(updateMean){
        LDREntriesLength++;
        LDREntriesTotal += LDRFront;
        meanLDR = (double)LDREntriesTotal/(double)LDREntriesLength;
    }
    previousLDRValues[previousLDRIndex] = LDRFront;
    previousLDRIndex  = (previousLDRIndex + 1)%DIFFERENTIATION_INTERVAL_HARVESTING;
}

void Robot::clearMeanLDR(){
    LDREntriesLength = 0;
    LDREntriesTotal = 0;
}

int Robot::getLDRFront(){
    return LDRFront;
}

bool Robot::getBitValueAsBoolean(int value, int bit){
    return (value & (int) pow(2,bit)) != 0;
}

int Robot::getDistanceSensorVoltage(){
    return distanceSensorVoltage;
}

void Robot::readPinValue(int port){
    switch(port){
        case 0:
            pinValues[port] = rlink.request(READ_PORT_0);
            break;
        case 1:
            pinValues[port] = rlink.request(READ_PORT_1);
            break;
        default:
            cout << "Error: Port " << port << " not in use." << endl;
            break;
    }
}

bool Robot::getStartButtonPressedWithUpdate(){
    readPinValue(1);
    return getStartButtonPressed();
}

bool Robot::getStartButtonPressed(){
    return getBitValueAsBoolean(pinValues[1], 5);
}

void Robot::awaitStartButtonPress(){
    delay(500);
    while(!getStartButtonPressedWithUpdate()){
        delay(200);
    }
    delay(500);
}

void Robot::turnMechanism(){
    stopMotors();
    setMechanismMotor(-1.0);
    delay(8500);
    setMechanismMotor(0.0);
    delay(1000);
    setMechanismMotor(1.0);
    delay(6720);
    setMechanismMotor(0.0);
}

void Robot::harvestBrassica(){
    activateHarvestMechanism();
    delay(800);
    deactivateHarvestMechanism();
    delay(800);
}
