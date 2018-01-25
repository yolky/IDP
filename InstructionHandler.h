#include "Robot.h"
#include "Instruction.h"

#ifndef INSTRUCTION_HANDLER_H
#define INSTRUCTION_HANDLER_H
class InstructionHandler{
	private:
	public:
		InstructionHandler();
		void runInstructions(Instruction instructions[]);
		void runInstruction(Instruction instruction);
};
#endif
