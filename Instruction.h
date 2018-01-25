#include "Robot.h"

#ifndef INSTRUCTION_H
#define INSTRUCTION_H
class Instruction{
	private:
		Robot robot;
	public:
		Instruction(Robot& robot, void(*_operation)(), bool(*_stopCondition)(), int _minTime, int _maxTime);
		void (*operation)();
		bool (*stopCondition)();
		int minTime;
		int maxTime;
};
#endif
