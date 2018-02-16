#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>
#include <functional>
#include "Robot.h"
#include "Robot.cpp"
#include "Instruction.h"
#include "Instruction.cpp"
#include "InstructionHandler.h"
#include "InstructionHandler.cpp"
#include "StopwatchWithPause.h"
#include "StopwatchWithPause.cpp"
#include "HarvestInstructionFactory.h"
#include "HarvestInstructionFactory.cpp"

#include "globals.cpp"

function<void()> makeTurn(Robot& robot, double leftMotorSpeed, double rightMotorSpeed);
function<void()> overBrassicaOperation(Robot& robot, double speed, double kP);
function<void()> followLineOperationWithUpdate(Robot& robot, double speed, double kP);
function<void()> getNegGainLFPostInstruction(Robot& robot, double speed, double kP, int duration);
function<bool ()> stopOnLargeBrassica(Robot& robot);
function<void(int previousValues[], int& previousValuesLength, int& previousValueIndex)> followDistanceSensor(Robot& robot, double speed, int targetDistance, double kP, double kD);
Instruction& getHarvestForwardInstructions();
void exitStartArea();
void turnAroundAndGetNearStart();
void turn180(bool turnRight);
void exitFirstDepositAreaToFlat();
void exitDepositeArea(bool firstStartingArea);
void nearHarvestAreaToDeposit(bool depositInFirstSection);
void exitSecondDepositAreaToStart();

int main()
{
    delay(500);
	int val = robot.sendTestInstruction();  // send test instruction=
	if (val == TEST_INSTRUCTION_RESULT) {     // check result
        robot.deactivateHarvestMechanism(true);
        delay(500);

        cout << &robot << endl;
        while(true){
            cout<< "Waiting for start button to be pressed" << endl;
            robot.awaitStartButtonPress();
            cout << "OPERATIONS STARTED" << endl;
            //instructionHandler.runInstruction(horribleIdea);
            exitStartArea();
            flatForwardHarvest(false);
            turnAroundAndGetNearStart();
            nearHarvestAreaToDeposit(true);
            exitFirstDepositAreaToFlat();
            flatForwardHarvest(true);
            turnAroundAndGetNearStart();
            nearHarvestAreaToDeposit(false);
            exitSecondDepositAreaToStart();
            robot.myfile.close();
            robot.myfile2.close();
        }
		return 0;                             // all OK, finish
	}
	else if (val == REQUEST_ERROR) {
		cout << "Fatal errors on link:" << endl;
		robot.printErrors();
	}
	else
		cout << "Test failed (bad value returned)" << endl;
	return -1;                                // error, finish

}

void exitStartArea(){
    Instruction startTurn(robot, makeTurn(robot, -0.4, 1.0), 1100, 10000);
    Instruction driveUpToFlatHarvest(robot, followLineOperation(robot, 0.7), 200, 10000, 0);

    startTurn.setStopCondition(STOP_CONDITION_LEFT_TURN);
    startTurn.setNextInstruction(driveUpToFlatHarvest);

    driveUpToFlatHarvest.setStopCondition(STOP_CONDITION_T_JUNCTION);
    driveUpToFlatHarvest.setPostInstruction(stopRobotPreInstruction);
    instructionHandler.runInstruction(startTurn);
}

void nearHarvestAreaToDeposit(bool depositInFirstSection){
    Instruction driveUp1(robot, followLineOperation(robot, 0.7), 600, 10000);
    Instruction driveUp2(robot, followLineOperation(robot, 0.7), 700, 10000);
    Instruction depositTurn(robot, makeTurn(robot, 0.0, 0.6), 4050, 10000);
    Instruction alignToFirstBoxReverse(robot, followLineOperation(robot, -0.4), 4400, 20000);
    Instruction alignToFirstBox1(robot, followDistanceSensor(robot, 0.4, 90, 0.007, 0.1), 6000, 40000);
    Instruction depositBrassicas1(robot, doNothingOperation(), 10, 10000);



    if(!depositInFirstSection){
        alignToFirstBox1.minTime = 21000;
    }

    driveUp1.setStopCondition(STOP_CONDITION_T_JUNCTION);
    driveUp1.setNextInstruction(driveUp2);
    driveUp2.setStopCondition(STOP_CONDITION_T_JUNCTION);
    driveUp2.setNextInstruction(depositTurn);
    depositTurn.setStopCondition(alwaysStopCondition);
    depositTurn.setNextInstruction(alignToFirstBoxReverse);

    alignToFirstBoxReverse.setStopCondition(alwaysStopCondition);
    alignToFirstBoxReverse.setNextInstruction(alignToFirstBox1);
    alignToFirstBox1.setStopCondition(STOP_CONDITION_FRONT_TWO);
    alignToFirstBox1.setNextInstruction(depositBrassicas1);

    function<void()> rotateMechanismPreInstruction = [&robot](){robot.turnMechanism();};
        depositBrassicas1.setPreInstruction(rotateMechanismPreInstruction);
        depositBrassicas1.setStopCondition(alwaysStopCondition);

    instructionHandler.runInstruction(driveUp1);
}


void turnAroundAndGetNearStart(){
    Instruction goStraight(robot, followLineOperation(robot, 0.7), 3000, 15000);
    goStraight.setStopCondition(STOP_CONDITION_T_JUNCTION);

    turn180(true);
    instructionHandler.runInstruction(goStraight);
}

void turn180(bool turnRight){
    Instruction tight901(robot, makeTurn(robot, 1.0, -0.25), 1300, 10000);
    Instruction shortReverse(robot, followLineOperation(robot, -0.6, -0.2), 1500, 5000);
    Instruction tight902(robot, makeTurn(robot, 1.0, -0.25), 1300, 10000);

    tight901.setStopCondition(STOP_CONDITION_RIGHT_TURN);
    tight901.setNextInstruction(shortReverse);

    shortReverse.setStopCondition(STOP_CONDITION_T_JUNCTION);
    shortReverse.setPostInstruction(stopRobotPreInstruction);
    shortReverse.setNextInstruction(tight902);

    tight902.setStopCondition(STOP_CONDITION_RIGHT_TURN);
    tight902.setPostInstruction(stopRobotPreInstruction);

    instructionHandler.runInstruction(tight901);
}

void exitFirstDepositAreaToFlat(){

    Instruction goStraight1(robot, followLineOperation(robot, 0.8), 1000, 15000);
    Instruction goStraight2(robot, followLineOperation(robot, 0.8), 400, 15000);

    Instruction turnRight(robot, makeTurn(robot, 1.0, -0.3), 1300, 10000);
    Instruction goStraightToHarvestArea(robot, followLineOperation(robot, 0.5), 3000, 15000);


    goStraight1.setStopCondition(STOP_CONDITION_T_JUNCTION);
    goStraight1.setNextInstruction(goStraight2);
    goStraight2.setStopCondition(STOP_CONDITION_T_JUNCTION);
    goStraight2.setNextInstruction(turnRight);
    turnRight.setStopCondition(STOP_CONDITION_RIGHT_TURN);
    turnRight.setNextInstruction(goStraightToHarvestArea);
    goStraightToHarvestArea.setStopCondition(STOP_CONDITION_T_JUNCTION);
    goStraightToHarvestArea.setPostInstruction(stopRobotPreInstruction);

    exitDepositeArea(true);

    instructionHandler.runInstruction(goStraight1);
}

void exitDepositeArea(bool firstStartingArea){
    Instruction alignToFirstBox1(robot, followDistanceSensor(robot, 0.4, 90, 0.007, 0.1), 400, 15000);
    Instruction wide90(robot, makeTurn(robot, 0.1, 0.7), 1500, 10000);
    Instruction tight901(robot, makeTurn(robot, -0.3, 1.0), 1300, 10000);
    Instruction shortReverse(robot, followLineOperation(robot, -0.7, -0.2), 80, 1000);
    Instruction tight902(robot, makeTurn(robot, -0.34, 1.0), 1300, 10000);

    alignToFirstBox1.setStopCondition(STOP_CONDITION_T_JUNCTION_VERY_FUZZY);
    alignToFirstBox1.setNextInstruction(tight901);

    tight901.setStopCondition(STOP_CONDITION_LEFT_TURN);
    tight901.setNextInstruction(shortReverse);

    shortReverse.setStopCondition(STOP_CONDITION_T_JUNCTION);
    shortReverse.setPostInstruction(stopRobotPreInstruction);
    shortReverse.setNextInstruction(tight902);
    shortReverse.setDefaultNextInstructionIndex(0);

    tight902.setStopCondition(STOP_CONDITION_LEFT_TURN);
    tight902.setPostInstruction(stopRobotPreInstruction);


    wide90.setStopCondition(STOP_CONDITION_LEFT_TURN);
    wide90.setNextInstruction(shortReverse);

    if(firstStartingArea){
        instructionHandler.runInstruction(alignToFirstBox1);
    }
    else{
        instructionHandler.runInstruction(wide90);
    }
}

void exitSecondDepositAreaToStart(){

    Instruction goStraight1(robot, followLineOperation(robot, 0.7), 5000, 15000);
    Instruction goStraight2(robot, followLineOperation(robot, 0.7), 3000, 15000);
    Instruction goStraight3(robot, followLineOperation(robot, 0.7), 700, 15000);

    goStraight1.setStopCondition(STOP_CONDITION_T_JUNCTION);
    goStraight1.setNextInstruction(goStraight2);
    goStraight2.setStopCondition(STOP_CONDITION_T_JUNCTION);
    goStraight2.setNextInstruction(goStraight3);
    goStraight3.setStopCondition(STOP_CONDITION_T_JUNCTION);
    goStraight3.setPostInstruction(stopRobotPreInstruction);

    exitDepositeArea(false);
    instructionHandler.runInstruction(goStraight1);
}

function<void()> getNegGainLFPostInstruction(Robot& robot, double speed, double kP, int duration){
    return [&, speed, kP, duration]{
        robot.stopMotors();
        stopwatch watch;
        watch.start();
        while(watch.read()<duration){
            robot.updateLineSensors();
            followLineOperation(robot, speed, kP)();
        }
    };
}

function<void()> makeTurn(Robot& robot, double leftMotorSpeed, double rightMotorSpeed) {
	return [&, leftMotorSpeed, rightMotorSpeed]() {
        robot.updateLineSensors();
        robot.setMotors(leftMotorSpeed, rightMotorSpeed);
	};
}

function<void(int previousValues[], int& previousValuesLength, int& previousValueIndex)> followDistanceSensor(Robot& robot, double speed, int targetDistance, double kP, double kD) {
	return [&, speed, kP, targetDistance, kD](int previousValues[], int& previousValuesLength, int& previousValueIndex) {
        robot.updateDistanceSensor();
        int currentDistance = robot.getDistanceSensorVoltage();
        double leftWheelExtra = kP * (targetDistance-currentDistance);
        int oldestValue = previousValues[(previousValueIndex +1)%previousValuesLength];
        int newestValue = previousValues[previousValueIndex];
        double averageChange = 0;
        if(oldestValue != -1){
            averageChange = ((double)newestValue - (double)oldestValue)/ (double) previousValuesLength;
        }
        leftWheelExtra -= kD* averageChange;
        robot.setMotors(speed * (1.0 + leftWheelExtra), speed * (1.0 - leftWheelExtra));
	};
}

