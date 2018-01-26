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
function<void()> followLineOperation(Robot& robot);

int main()
{
	Robot robot;


	//const function <bool()> STOP_CONDITION_RIGHT_45 = getLineSensorStopCondition(robot, { -1, -1, 0, 1 });
	//const function <bool()> STOP_CONDITION_LEFT_45 = getLineSensorStopCondition(robot, { 1, 0, -1, -1 });
	//const function <bool()> STOP_CONDITION_RIGHT_TURN = getLineSensorStopCondition(robot, { -1, -1, 1, 1 });
	//const function <bool()> STOP_CONDITION_LEFT_TURN = getLineSensorStopCondition(robot, { 1, 1, -1, -1 });



    delay(10000);
	int val = robot.sendTestInstruction();  // send test instruction
	if (val == TEST_INSTRUCTION_RESULT) {     // check result

        int SENSOR_T_JUNCTION [4] =  {1, 1, 1, 1 };
        cout << SENSOR_T_JUNCTION[0] << endl;
        const function <bool()> STOP_CONDITION_T_JUNCTION = getLineSensorStopCondition(robot, SENSOR_T_JUNCTION);

        Instruction testInstruction(robot, followLineOperation(robot), STOP_CONDITION_T_JUNCTION, 1000, 20000000);
        Instruction testInstruction2(robot, followLineOperation(robot), STOP_CONDITION_T_JUNCTION, 1000, 20000000);
        Instruction testInstruction3(robot, followLineOperation(robot), STOP_CONDITION_T_JUNCTION, 1000, 20000000);

        testInstruction.operation();
        Instruction instructions [3] = {testInstruction,testInstruction2, testInstruction3};

		cout << "Test passed" << endl;

		InstructionHandler instructionHandler;
		instructionHandler.runInstructions(instructions);

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

function<void()> followLineOperation(Robot& robot) {
	return [&]() {
        int goingStraight [4] = {-1, 0, 0, -1};
        int offLeft [4] = {-1, 1, 0, -1};
        int offRight [4] = {-1, 0, 1, -1};
		if (robot.checkLineSensorsMatch(goingStraight)) {
            //cout << "GOING STRAIGHT" << endl;
			robot.setMotors(1.0, 1.0);
		}
		else if (robot.checkLineSensorsMatch(offLeft)) {
            //cout << "OFF LEFT" << endl;
			robot.setMotors(0.8, 1.0);
		}
		else if (robot.checkLineSensorsMatch(offRight)) {
            //cout << "OFF RIGHT" << endl;
			robot.setMotors(1.0, 0.8);
		}
	};
}
