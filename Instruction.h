#include "Robot.h"

#ifndef INSTRUCTION_H
#define INSTRUCTION_H
class Instruction{
	private:
		Robot robot;
	public:
		Instruction(Robot& robot, void(*_operation)());
		void (*operation)();
};
#endif
