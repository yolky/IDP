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

void bullShit();
function<bool()> getLineSensorStopCondition(Robot& robot, int sensorState[]);
function<bool ()> getLineSensorStopConditionFuzzy(Robot& robot, int sensorState[], int interval);
function<void(int& turnLeftPoints)> followLineOperation(Robot& robot, double speed, double kP = 0.2);
function<void(int& turnLeftPoints)> turnLeft(Robot& robot, double turnSpeed) ;

int main()
{
	Robot robot;


	//const function <bool()> STOP_CONDITION_RIGHT_45 = getLineSensorStopCondition(robot, { -1, -1, 0, 1 });
	//const function <bool()> STOP_CONDITION_LEFT_45 = getLineSensorStopCondition(robot, { 1, 0, -1, -1 });
	//const function <bool()> STOP_CONDITION_RIGHT_TURN = getLineSensorStopCondition(robot, { -1, -1, 1, 1 });
	//const function <bool()> STOP_CONDITION_LEFT_TURN = getLineSensorStopCondition(robot, { 1, 1, -1, -1 });



    delay(3000);
	int val = robot.sendTestInstruction();  // send test instruction=
	if (val == TEST_INSTRUCTION_RESULT) {     // check result

        int SENSOR_T_JUNCTION [4] =  {1, -1, -1, 1 };
        int SENSOR_TEST[4] = {-1,1,-1,-1};
        cout << SENSOR_T_JUNCTION[0] << endl;
        const function <bool()> STOP_CONDITION_T_JUNCTION = getLineSensorStopConditionFuzzy(robot, SENSOR_T_JUNCTION,300);
        const function <bool()> STOP_CONDITION_LEFT_TURN = getLineSensorStopCondition(robot, SENSOR_TEST);

        Instruction testInstruction1(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1000, 20000000);
        Instruction testInstruction2(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1000, 20000000);
        Instruction testInstruction3(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1000, 20000000);

        Instruction testInstruction4(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1000, 20000000);

        Instruction fuckHills(robot, followLineOperation(robot, 0.8, 0.4), STOP_CONDITION_T_JUNCTION, 1000, 20000000); //THE HILL
        Instruction downHill(robot, followLineOperation(robot, 0.8, 0.7), STOP_CONDITION_T_JUNCTION, 1000, 20000000); //THE HILL
        Instruction testInstruction5(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1000, 20000000);

        Instruction testInstruction6(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1000, 20000000);
        Instruction testInstruction7(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1000, 20000000);
        Instruction testInstruction8(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1500, 20000000);
        Instruction testInstruction9(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 1000, 20000000);
        Instruction testInstruction10(robot, followLineOperation(robot, 1.0), STOP_CONDITION_T_JUNCTION, 800, 20000000);




        Instruction flatTurn(robot, turnLeft(robot, 1.0), STOP_CONDITION_LEFT_TURN, 2100, 20000000, 0);
        Instruction hillTurn(robot, turnLeft(robot, 0.7), STOP_CONDITION_LEFT_TURN, 2000, 20000000, 0);

        Instruction instructions [15] = {testInstruction1, testInstruction2, testInstruction3, flatTurn, testInstruction4, fuckHills, hillTurn, downHill, testInstruction6, testInstruction7, flatTurn, testInstruction8, testInstruction9, testInstruction10, flatTurn};

		cout << "Test passed" << endl;

		InstructionHandler instructionHandler;
		instructionHandler.runInstructions(instructions, 15);
        instructionHandler.runInstructions(instructions, 15);

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

void bullShit(){
	cout<<"something belse" << endl;
}

function<bool ()> getLineSensorStopCondition(Robot& robot, int sensorState[]) {
	return [&, sensorState]() {
		robot.updateLineSensors();
        return robot.checkLineSensorsMatch(sensorState);
	};
}


function<bool ()> getLineSensorStopConditionFuzzy(Robot& robot, int sensorState[], int interval) {
	return [&, sensorState, interval]() {
		robot.updateLineSensors();
        return robot.checkLineSensorsMatchFuzzy(sensorState, interval);
	};
}

function<void(int& turnLeftPoints)> followLineOperation(Robot& robot, double speed, double kP) {
	return [&, speed, kP](int& turnLeftPoints) {
        double INTEGRAL_GAIN = 0.001;
        double fuckMe = INTEGRAL_GAIN * (double)turnLeftPoints;
        //cout<< fuckMe << endl;
        //cout<< turnLeftPoints << endl;
        int goingStraight [4] = {-1, 0, 0, -1};
        int offLeft [4] = {-1, 1, 0, -1};
        int offRight [4] = {-1, 0, 1, -1};
		if (robot.checkLineSensorsMatch(goingStraight)) {
            //cout << "GOING STRAIGHT" << endl;
			robot.setMotors(speed*(1.0 - fuckMe), speed*(1.0 + fuckMe));
		}
		else if (robot.checkLineSensorsMatch(offLeft)) {
            turnLeftPoints ++;
            //cout << "OFF LEFT" << endl;
			robot.setMotors(speed*(1.0 - kP -fuckMe), speed*(1.0 + fuckMe));
		}
		else if (robot.checkLineSensorsMatch(offRight)) {
            turnLeftPoints --;
            //cout << "OFF RIGHT" << endl;
			robot.setMotors(speed*(1.0 - fuckMe), speed*(1.0 - kP + fuckMe));
		}
	};
}


function<void(int& turnLeftPoints)> turnLeft(Robot& robot, double turnSpeed) {
	return [&, turnSpeed](int& turnLeftPoints) {
        robot.setMotors(0.0*turnSpeed, 1.0*turnSpeed);
	};
}
