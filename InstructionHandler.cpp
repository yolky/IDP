#include <robot_delay.h>
#include <stopwatch.h>
#include "Instruction.h"

InstructionHandler::InstructionHandler(){
};

void InstructionHandler::runInstructions(Instruction instructions[]){
	for (int i = 0; i < sizeof(instructions); i++) {
		runInstruction(instructions[i])
	}
}

void InstructionHandler::runInstruction(Instruction instruction) {
	Stopwatch watch;
	watch.start();
	int runningTime = watch.read();
	while ((runningTime < instruction.maxTime && !instruction.stopCondition()) || runningTime < instruction.minTime) {
		instructions.operation();
		runningTime = watch.read();
	}
	if (runningTime > instruction.maxTime) {
		//do reversing shit
	}
	watch.stop();
}
