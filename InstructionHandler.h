#include "Robot.h"
#include "Instruction.h"

#ifndef INSTRUCTION_HANDLER_H
#define INSTRUCTION_HANDLER_H
class InstructionHandler{
	private:
        Robot& robot;
	public:
		InstructionHandler(Robot& _robot);
		void runInstruction(Instruction& instruction);
};
#endif
