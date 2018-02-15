#include <robot_delay.h>
#include <stopwatch.h>
#include "InstructionHandler.h"
#include "Instruction.h"

InstructionHandler::InstructionHandler(Robot& _robot):
robot(_robot){
};

void InstructionHandler::runInstruction(Instruction& instruction) {
    cout << "starting int" << endl;
    instruction.startRunning();
    int runningTime = instruction.getRunningTime();
    if(instruction.hasPreInstruction){
        instruction.preInstruction();
    }
cout << "starting operation" << endl;
	while (((runningTime < instruction.maxTime && !instruction.evaluateStopConditions()) || runningTime < instruction.minTime) && !robot.getStartButtonPressedWithUpdate()) {
		instruction.operation();
		runningTime = instruction.getRunningTime();
		delay(2);
	}

	int stopTime = runningTime;
	while(runningTime < stopTime+instruction.extraTime){
        runningTime = instruction.getRunningTime();
        instruction.operation();
        delay(2);
	}
    instruction.stopRunning();
	if (runningTime > instruction.maxTime) {
        //do reversing shit
	}
	if(instruction.hasPostInstruction){
        instruction.postInstruction();
    }


    if(robot.getStartButtonPressed() || instruction.isEndState){
        if(instruction.isEndState){
            cout << "end of instructions reached" << endl;
        }
        else{
            cout << "Stop button pressed" << endl;
        }
        robot.stopMotors();
	}
	else {
        runInstruction(instruction.getNextInstruction());
	}
}
