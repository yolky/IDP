#include <robot_delay.h>
#include "Instruction.h"

InstructionHandler::InstructionHandler(){
};

void InstructionHandler::runInstructions(Instruction instructions[]){
	for(int i=0; i< 10; i++){
		instructions[0].operation();
		delay(500);
	}
}
