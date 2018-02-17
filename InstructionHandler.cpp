#include <robot_delay.h>
#include <stopwatch.h>
#include "InstructionHandler.h"
#include "Instruction.h"

InstructionHandler::InstructionHandler(Robot& _robot):
robot(_robot){
};

void InstructionHandler::runInstruction(Instruction& instruction) {
    instruction.startRunning();
    int runningTime = instruction.getRunningTime();
    if(instruction.hasPreInstruction){
        instruction.preInstruction();
    }
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

	if(instruction.hasPostInstruction){
        instruction.postInstruction();
    }

    if (runningTime > instruction.maxTime && instruction.hasDefaultInstruction) {
        runInstruction(instruction.getDefaultNextInstruction());
	}
    else if(robot.getStartButtonPressed() || instruction.isEndState){
        if(instruction.isEndState){
            cout << "end of instructions reached" << endl;
        }
        else{
            cout << "Stop button pressed" << endl;
        }
        robot.stopMotors();
        robot.harvestTimer.stop();
        robot.clearMeanLDR();
	}
	else {
        runInstruction(instruction.getNextInstruction());
	}
}
