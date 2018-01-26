#include <robot_delay.h>
#include <stopwatch.h>
#include "InstructionHandler.h"
#include "Instruction.h"

InstructionHandler::InstructionHandler(){
};

void InstructionHandler::runInstructions(Instruction instructions[]){
	for (int i = 0; i < sizeof(instructions); i++) {
        cout << "al;skdjf;alsdf" << endl;
		runInstruction(instructions[i]);
	}
}

void InstructionHandler::runInstruction(Instruction& instruction) {
	stopwatch watch;
	watch.start();
	int runningTime = watch.read();
	while ((runningTime < instruction.maxTime && !instruction.stopCondition()) || runningTime < instruction.minTime) {
        cout<< runningTime << endl;
		instruction.operation();
		runningTime = watch.read();
		delay(20);
	}
	if (runningTime > instruction.maxTime) {
		//do reversing shit
	}
	watch.stop();
}
