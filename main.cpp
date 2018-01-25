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
function<bool()> getLineSensorStopCondition(Robot robot, int sensorState[]);
function<void()> followLineOperation(Robot robot);

int main()
{
	Robot robot;

	const function <bool()> STOP_CONDITION_T_JUNCTION = getLineSensorStopCondition(robot, { 1, 1, 1, 1 });
	const function <bool()> STOP_CONDITION_RIGHT_45 = getLineSensorStopCondition(robot, { -1, -1, 0, 1 });
	const function <bool()> STOP_CONDITION_LEFT_45 = getLineSensorStopCondition(robot, { 1, 0, -1, -1 });
	const function <bool()> STOP_CONDITION_RIGHT_TURN = getLineSensorStopCondition(robot, { -1, -1, 1, 1 });
	const function <bool()> STOP_CONDITION_LEFT_TURN = getLineSensorStopCondition(robot, { 1, 1, -1, -1 });


	Instruction testInstruction(robot, followLineOperation(robot), STOP_CONDITION_T_JUNCTION, 1000, 20000);
	testInstruction.operation();
	Instruction instructions [1] = {testInstruction};
	
	function <bool ()> = getLine
	
	int val = robot.sendTestInstruction();  // send test instruction
	if (val == TEST_INSTRUCTION_RESULT) {     // check result
		cout << "Test passed" << endl;
		
		InstructionHandler instructionHandler;
		instructionHandler.runInstructions(instructions);

		return 0;                             // all OK, finish
	}
	else if (val == REQUEST_ERROR) {
		cout << "Fatal errors on link:" << endl;
	}
	else
		cout << "Test failed (bad value returned)" << endl;
	return -1;                                // error, finish

}

void bullShit(){
	cout<<"something belse" << endl;
}

function<bool ()> getLineSensorStopCondition(Robot robot, int sensorState[]) {
	return []() {
		robot.updateLineSensors();
		robot.checkLineSensorsMatch(sensorState);
	};
}

function<void()> followLineOperation(Robot robot) {
	return []() {
		if (robot.checkLineSensorsMatch([-1, 0, 0, -1])) {
			robot.setMotors(1.0, 1.0);
		}
		else if (robot.checkLineSensorsMatch([-1, 1, 0, -1])) {
			robot.setMotors(1.0, 0.8);
		}
		else if (robot.checkLineSensorsMatch([-1, 0, 1, -1])) {
			robot.setMotors(0.8, 1.0);
		}
	}
}