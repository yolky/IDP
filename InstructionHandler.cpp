#include <robot_delay.h>
#include <stopwatch.h>
#include "InstructionHandler.h"
#include "Instruction.h"

InstructionHandler::InstructionHandler(){
};

void InstructionHandler::runInstructions(Instruction instructions[], int length){
	for (int i = 0; i < length; i++) {
        cout << "al;skdjf;alsdf" << endl;
		runInstruction(instructions[i]);
	}
	cout << "done" << endl;
}

void InstructionHandler::runInstruction(Instruction& instruction) {
	stopwatch watch;
	watch.start();
	int runningTime = watch.read();
	while ((runningTime < instruction.maxTime && !instruction.stopCondition()) || runningTime < instruction.minTime) {
		instruction.operation();
		runningTime = watch.read();
		delay(10);
	}
	cout << instruction.minTime << endl;
	if (runningTime > instruction.maxTime) {
		//do reversing shit
	}
	else{
        cout<< instruction.extraTime<< endl;
        delay(instruction.extraTime);
	}
	watch.stop();
}
