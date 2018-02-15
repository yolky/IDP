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

#include "globals.cpp"

function<void()> followLineOperation(Robot& robot, double speed, double kP = 0.2, double leftWheelOffset = 0.0 , double rightWheelOffset = 0.0);
function<void()> makeTurn(Robot& robot, double leftMotorSpeed, double rightMotorSpeed);
function<bool()> aboveAverageIncreaseRate(Robot& robot, double threshold, bool greaterThan);
function<void()> doNothingOperation(Robot& robot);
function<void()> overBrassicaOperation(Robot& robot, double speed, double kP);
function<bool()> aboveMeanStopCondition(Robot& robot, double threshold, bool doUpdateMean, bool greaterThan);
function<void()> followLineOperationWithUpdate(Robot& robot, double speed, double kP);
function<bool()> alwaysStop();
function<void()> getNegGainLFPostInstruction(Robot& robot, double speed, double kP, int duration);
bool testStop();
function<bool ()> stopOnLargeBrassica(Robot& robot);
function<void(int previousValues[], int& previousValuesLength, int& previousValueIndex)> followDistanceSensor(Robot& robot, double speed, int targetDistance, double kP, double kD);
Instruction& getHarvestForwardInstructions();

InstructionHandler instructionHandler(robot);

int main()
{


	//const function <bool()> STOP_CONDITION_RIGHT_45 = getLineSensorStopCondition(robot, { -1, -1, 0, 1 });
	//const function <bool()> STOP_CONDITION_LEFT_45 = getLineSensorStopCondition(robot, { 1, 0, -1, -1 });
	//const function <bool()> STOP_CONDITION_RIGHT_TURN = getLineSensorStopCondition(robot, { -1, -1, 1, 1 });
	//const function <bool()> STOP_CONDITION_LEFT_TURN = getLineSensorStopCondition(robot, { 1, 1, -1, -1 });


    delay(500);
	int val = robot.sendTestInstruction();  // send test instruction=
	if (val == TEST_INSTRUCTION_RESULT) {     // check result





        robot.activateHarvestMechanism();
        delay(1000);
        robot.deactivateHarvestMechanism();
        delay(500);

        //cout << "start button "<< robot.getStartButtonPressedWithUpdate() << endl;



        Instruction& beginHarvest(getHarvestForwardInstructions());


        instructionHandler.runInstruction(beginHarvest);


        Instruction startTurn(robot, makeTurn(robot, -0.25, 0.8), 1700, 10000);
        Instruction lap1(robot, followLineOperation(robot, 0.7), 600, 10000);
        Instruction lap2(robot, followLineOperation(robot, 0.7), 600, 10000);
        Instruction rightTurn1(robot, makeTurn(robot, 0.7, 0.15), 2300, 10000);
        Instruction lap3(robot, followLineOperation(robot, 0.7), 600, 10000);
        Instruction climbHill(robot, followLineOperation(robot, 0.7), 600, 10000,3000);
        Instruction left45(robot, makeTurn(robot, 0.10, 0.7), 1000, 9000);
        Instruction reverseDownhill(robot, followLineOperation(robot, -0.3, -0.2, 0.2), 20000, 100000000);

        Instruction trackStand(robot, makeTurn(robot, 0.2, 0.0), 7000, 10000);


        startTurn.setStopCondition(STOP_CONDITION_LEFT_TURN);
        startTurn.setNextInstruction(lap1);

        lap1.setStopCondition(STOP_CONDITION_T_JUNCTION);
        lap1.setNextInstruction(lap2);

        lap2.setStopCondition(STOP_CONDITION_T_JUNCTION);
        lap2.setNextInstruction(rightTurn1);

        rightTurn1.setStopCondition(STOP_CONDITION_RIGHT_TURN);
        rightTurn1.setNextInstruction(lap3);

        lap3.setStopCondition(STOP_CONDITION_T_JUNCTION);
        lap3.setNextInstruction(climbHill);

        climbHill.setStopCondition(STOP_CONDITION_FAR_RIGHT);
        climbHill.setNextInstruction(left45);

        left45.setStopCondition(STOP_CONDITION_LEFT_TURN);
        left45.setNextInstruction(reverseDownhill);

        reverseDownhill.setStopCondition(alwaysStopCondition);
        trackStand.setStopCondition(alwaysStopCondition);

        Instruction dogPoo1(robot, followLineOperation(robot, 0.7), 600, 10000, 3000);
        Instruction largeRightTurn(robot, makeTurn(robot, 0.5, 0.2), 2000, 2000);

        dogPoo1.setStopCondition(STOP_CONDITION_T_JUNCTION);
        dogPoo1.setNextInstruction(largeRightTurn);

        largeRightTurn.setStopCondition(STOP_CONDITION_T_JUNCTION);

        Instruction alignToFirstBoxReverse(robot, followLineOperation(robot, -0.4), 4000, 20000);
        Instruction alignToFirstBox1(robot, followDistanceSensor(robot, 0.4, 100, 0.01, 0.1), 500, 20000);
        Instruction depositBrassicas1(robot, doNothingOperation(robot), 10, 10000);

        Instruction driveUp1(robot, followLineOperation(robot, 0.7), 600, 10000);
        Instruction driveUp2(robot, followLineOperation(robot, 0.7), 500, 10000);
        Instruction driveUp3(robot, followLineOperation(robot, 0.7), 500, 10000);
        Instruction depositTurn(robot, makeTurn(robot, -0.1, 0.6), 3700, 10000);

        driveUp1.setStopCondition(STOP_CONDITION_T_JUNCTION);
        driveUp1.setNextInstruction(driveUp2);
        driveUp2.setStopCondition(STOP_CONDITION_T_JUNCTION);
        driveUp2.setNextInstruction(driveUp3);
        driveUp3.setStopCondition(STOP_CONDITION_T_JUNCTION);
        driveUp3.setNextInstruction(depositTurn);
        depositTurn.setStopCondition(alwaysStopCondition);
        depositTurn.setNextInstruction(alignToFirstBoxReverse);

        alignToFirstBoxReverse.setStopCondition(alwaysStopCondition);
        alignToFirstBoxReverse.setNextInstruction(alignToFirstBox1);
        alignToFirstBox1.setStopCondition(STOP_CONDITION_T_JUNCTION);
        alignToFirstBox1.setNextInstruction(depositBrassicas1);

        function<void()> rotateMechanismPreInstruction = [&robot](){robot.turnMechanism();};
        depositBrassicas1.setPreInstruction(rotateMechanismPreInstruction);
        depositBrassicas1.setStopCondition(alwaysStopCondition);

        cout << &robot << endl;
        while(true){
            cout<< "Waiting for start button to be pressed" << endl;
            robot.awaitStartButtonPress();
            cout << "OPERATIONS STARTED" << endl;
            instructionHandler.runInstruction(driveUp1);
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

Instruction& getHarvestForwardInstructions(){
    Instruction beginHarvest(robot, followLineOperation(robot, 0.3, 0.2), 100, 0);
    Instruction preBrassica(robot, followLineOperation(robot, 0.3, 0.2), 10, 10000);
    Instruction overCauliflower(robot, overBrassicaOperation(robot, 0.3, 0.2),30, 10000);
    Instruction overCabbage(robot, overBrassicaOperation(robot, 0.3, 0.2),200, 10000);
    Instruction activateMechanism(robot, doNothingOperation(robot), 20,10000);


    function<void()> clearMeanFunction = [&robot](){robot.clearMeanLDR();};
    function<bool()> beforeStripStop = aboveMeanStopCondition(robot, -10, true, false);
    beginHarvest.setStopCondition(beforeStripStop);
    beginHarvest.setPostInstruction(clearMeanFunction);
    beginHarvest.setNextInstruction(preBrassica);


    function<bool()> preBrassicaStopCauliflower = aboveMeanStopCondition(robot, -9, true, false);
    function<bool()> preBrassicaStopCabbage = aboveMeanStopCondition(robot, 18, true, true);
    function<bool()> preBrassicaStopToHarvest = [&](){
        return (robot.brassicaHarvestTimes.front()!= 0 && (robot.harvestTimer.read() > robot.brassicaHarvestTimes.front()));
    };
    function<bool()>* preBrassicaStopConditions[] = {&preBrassicaStopCauliflower, &preBrassicaStopCabbage, &preBrassicaStopToHarvest};
    Instruction* preBrassicaNextInstructions[] = {&overCauliflower, &overCabbage, &activateMechanism};
    preBrassica.setStopConditions(preBrassicaStopConditions, 3);
    preBrassica.setNextInstructions(preBrassicaNextInstructions, 3);
    function<void()> preBrassicaPreFunction = [&robot](){
        cout << "mean cleared ?? " << endl;
        robot.harvestTimer.start();
    };
    preBrassica.setPreInstruction(preBrassicaPreFunction);

    function<bool()> overCauliflowerStopCondition = aboveMeanStopCondition(robot, -7, false, true);;
    function<void()> recordStartTime = [&robot](){
        //cout << "brassica start" << endl;
        robot.brassicaStartTime = robot.harvestTimer.read();
        robot.brassicaMaxValue = 0;
        robot.brassicaMinValue = 999;
    };
    function<void()> analyseBrassicaValues = [&robot](){
        robot.brassicaEndTime = robot.harvestTimer.read();
        int brassicaWidth = robot.brassicaEndTime - robot.brassicaStartTime;
        int brassicaMax = robot.brassicaMaxValue - robot.meanLDR;
        int brassicaMin = robot.brassicaMinValue - robot.meanLDR;
        //cout<< "brassica max: " << brassicaMax << endl;


        if(brassicaMin < -20 && brassicaWidth > 240){

            //cout << "LARGE CAULIFLOWER" << endl;
            robot.brassicaHarvestTimes.push(robot.brassicaStartTime + 1400);
        }

        if(brassicaMax > 38){
            //cout << "LARGE CABBAGE" << endl;
            robot.brassicaHarvestTimes.push(robot.brassicaStartTime + 1500);
        }
        robot.clearMeanLDR();
    };
    overCauliflower.setStopCondition(overCauliflowerStopCondition);
    overCauliflower.setNextInstruction(preBrassica);
    overCauliflower.setPreInstruction(recordStartTime);
    overCauliflower.setPostInstruction(analyseBrassicaValues);

    function<bool()> overCabbageStopCondition = aboveMeanStopCondition(robot, 2, false, false);
    overCabbage.setStopCondition(overCabbageStopCondition);
    overCabbage.setNextInstruction(preBrassica);
    overCabbage.setPreInstruction(recordStartTime);
    overCabbage.setPostInstruction(analyseBrassicaValues);


    function<void()> activateMechanismPreInstruction = [&robot](){
        robot.harvestTimer.pause();
        robot.stopMotors();
        delay(200);
        robot.brassicaHarvestTimes.pop();
        cout << "harvesting!!" << endl;
        robot.harvestBrassica();
    };
    function<void()> startHarvestTimer = [&robot](){
        robot.harvestTimer.start();
    };
    activateMechanism.setPreInstruction(activateMechanismPreInstruction);
    activateMechanism.setPostInstruction(startHarvestTimer);
    activateMechanism.setNextInstruction(preBrassica);
    activateMechanism.setStopCondition(alwaysStopCondition);

    cout << &*(beginHarvest.stopConditions[0]) << endl;
    return beginHarvest;
}


function<void()> doNothingOperation(Robot& robot){
    return [&robot](){
        delay(10);
    };
}

function<bool ()> stopOnLargeBrassica(Robot& robot){
    return [&]() {
		robot.updateLDRFront();
        return (robot.getLDRFront() > 190);
	};
}

function<bool ()> stopOnLargeCabbage(Robot& robot){
    return [&]() {
		robot.updateLDRFront();
        return (robot.getLDRFront() > 150);
	};
}



function<void()> followLineOperation(Robot& robot, double speed, double kP, double leftWheelOffset, double rightWheelOffset) {
	return [&, speed, kP, leftWheelOffset, rightWheelOffset]() {
        robot.updateLineSensors();
        int goingStraight [4] = {-1, 0, 0, -1};
        int offRight [4] = {-1, 1, 0, -1};
        int offLeft [4] = {-1, 0, 1, -1};
        if (robot.checkLineSensorsMatch(offRight)) {
			robot.setMotors(speed*(1.0 - kP) + leftWheelOffset, speed * (1.0 + kP) + rightWheelOffset);
		}
		else if (robot.checkLineSensorsMatch(offLeft)) {
			robot.setMotors(speed* (1.0 + kP) + leftWheelOffset, speed*(1.0 - kP) + rightWheelOffset);
		}
		else {
			robot.setMotors(speed + leftWheelOffset, speed + rightWheelOffset);
		}
	};
}

function<void()> followLineOperationWithUpdate(Robot& robot, double speed, double kP) {
	return [&, speed, kP]() {
        followLineOperation(robot, speed, kP)();
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

function<void()> overBrassicaOperation(Robot& robot, double speed, double kP){
    return [&, speed, kP](){
        followLineOperation(robot, speed, kP)();
        robot.brassicaMaxValue = max(robot.brassicaMaxValue, robot.getLDRFront());
        robot.brassicaMinValue = min(robot.brassicaMinValue, robot.getLDRFront());
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


function<bool()> aboveAverageIncreaseRate(Robot& robot, double threshold, bool greaterThan){
    return [&, threshold, greaterThan](){
        robot.updateLDRFront(false);
        int newestValue = robot.previousLDRValues[(robot.previousLDRIndex +1)%robot.DIFFERENTIATION_INTERVAL_HARVESTING];
        int oldestValue = robot.previousLDRValues[robot.previousLDRIndex];
        double averageChange = 0;
        if(oldestValue != -1){
            averageChange = ((double)newestValue - (double)oldestValue)/ (double) robot.DIFFERENTIATION_INTERVAL_HARVESTING;
        }
        return greaterThan? averageChange > threshold : averageChange < threshold;
    };
}
