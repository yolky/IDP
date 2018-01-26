#include "Robot.h"
#include <functional>

#ifndef INSTRUCTION_H
#define INSTRUCTION_H
class Instruction{
	private:
		Robot robot;
	public:
		//Instruction(Robot& robot, void(*_operation)(), bool(*_stopCondition)(), int _minTime, int _maxTime);
		Instruction(Robot& _robot, function<void ()> _operation, function<bool ()> _stopCondition, int _minTime, int _maxTime);
		//void (*operation)();
		//bool (*stopCondition)();
		function<void ()> operation;
		function<bool ()> stopCondition;
		int minTime;
		int maxTime;
};
#endif
