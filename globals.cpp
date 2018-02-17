#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED

#include <functional>
#include "InstructionHandler.h"
using namespace std;

function<bool()> getLineSensorStopCondition(Robot& robot, int sensorState[]);
function<bool()> getLineSensorStopConditionFuzzy(Robot& robot, int sensorState[], int interval);
function<bool()> alwaysStop();
function<void()> followLineOperation(Robot& robot, double speed, double kP = 0.2, double leftWheelOffset = 0.0, double rightWheelOffset = 0.0, double kI = 0.0);
function<void()> doNothingOperation(Robot& robot);
function<bool()> aboveMeanStopCondition(Robot& robot, double threshold, bool doUpdateMean, bool greaterThan);

Robot robot;

InstructionHandler instructionHandler(robot);

int SENSOR_T_JUNCTION [4] =  {1, -1, -1, 1 };
int SENSOR_LEFT_TURN[4] = {-1,1,-1,-1};
int SENSOR_RIGHT_TURN[4] = {-1,-1,1,-1};
int SENSOR_FAR_RIGHT[4] = {-1, -1, -1, 1};
int SENSOR_FAR_LEFT[4] = {1, -1, -1, -1};
int SENSOR_FRONT_TWO[4] = {-1, 1, 1, -1};

function <bool()> STOP_CONDITION_T_JUNCTION = getLineSensorStopConditionFuzzy(robot, SENSOR_T_JUNCTION,130);
function <bool()> STOP_CONDITION_T_JUNCTION_VERY_FUZZY = getLineSensorStopConditionFuzzy(robot, SENSOR_T_JUNCTION,400);
function <bool()> STOP_CONDITION_LEFT_TURN = getLineSensorStopCondition(robot, SENSOR_LEFT_TURN);
function <bool()> STOP_CONDITION_RIGHT_TURN = getLineSensorStopCondition(robot, SENSOR_RIGHT_TURN);
function <bool()> STOP_CONDITION_FAR_RIGHT = getLineSensorStopCondition(robot, SENSOR_FAR_RIGHT);
function <bool()> STOP_CONDITION_FRONT_TWO = getLineSensorStopConditionFuzzy(robot, SENSOR_FRONT_TWO, 200);
function<bool()> alwaysStopCondition = alwaysStop();

function<bool ()> getLineSensorStopConditionFuzzy(Robot& robot, int sensorState[], int interval) {
	return [&, sensorState, interval]() {
		robot.updateLineSensors();
        return robot.checkLineSensorsMatchFuzzy(sensorState, interval);
	};
}

function<bool ()> getLineSensorStopCondition(Robot& robot, int sensorState[]) {
	return [&, sensorState]() {
        return robot.checkLineSensorsMatch(sensorState);
	};
}

function<void()> stopRobotPreInstructionFactory(Robot& robot) {
	return [&robot]() {
        robot.stopMotors();
	};
}

function<void()> resetIntegralPreInstructionFactor(Robot& robot) {
	return [&robot]() {
        robot.resetIntegral();
	};
}

function<void()> stopRobotPreInstruction = stopRobotPreInstructionFactory(robot);
function<void()> resetIntegralPreInstruction = resetIntegralPreInstructionFactor(robot);

function<bool()> alwaysStop(){
    return [](){
        return true;
    };
}

function<void()> doNothingOperation(){
    return [](){
        delay(10);
    };
}

function<void()> followLineOperation(Robot& robot, double speed, double kP, double leftWheelOffset, double rightWheelOffset, double kI) {
	return [&, speed, kP, leftWheelOffset, rightWheelOffset, kI]() {
        robot.updateLineSensors();
        int goingStraight [4] = {-1, 0, 0, -1};
        int offRight [4] = {-1, 1, 0, -1};
        int offLeft [4] = {-1, 0, 1, -1};
        double integralOffset = (kI * (double) robot.integralCount);
        if(kI != 0.0){
            integralOffset = min(0.22, integralOffset);
            integralOffset = max(-0.22, integralOffset);
        }
        if (robot.checkLineSensorsMatch(offRight)) {
            robot.integralCount++;
			robot.setMotors(speed*(1.0 - kP - integralOffset) + leftWheelOffset, speed * (1.0 + kP + integralOffset) + rightWheelOffset);
		}
		else if (robot.checkLineSensorsMatch(offLeft)) {
            robot.integralCount--;
			robot.setMotors(speed* (1.0 + kP - integralOffset) + leftWheelOffset, speed*(1.0 - kP + integralOffset) + rightWheelOffset);
		}
		else {
			robot.setMotors(speed + leftWheelOffset - integralOffset, speed + rightWheelOffset + integralOffset);
		}
	};
}

function<bool()> aboveMeanStopCondition(Robot& robot, double threshold, bool doUpdateMean, bool greaterThan){
    return [&, threshold, doUpdateMean, greaterThan](){
        robot.updateLDRFront(doUpdateMean);
        if(greaterThan){
            return robot.getLDRFront() > robot.meanLDR + threshold;
        }
        else{
            return robot.getLDRFront() < robot.meanLDR + threshold;
        }
    };
}

#endif
