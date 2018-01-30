#include "Robot.h"
#include "Instruction.h"

#ifndef INSTRUCTION_HANDLER_H
#define INSTRUCTION_HANDLER_H
class InstructionHandler{
	private:
	public:
        Robot robot;
		InstructionHandler();
		void runInstructions(Instruction instructions[], int length);
		void runInstruction(Instruction& instruction);
};
#endif
