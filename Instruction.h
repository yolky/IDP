#include "Robot.h"
#include <functional>

#ifndef INSTRUCTION_H
#define INSTRUCTION_H
class Instruction{
	private:
		Robot robot;
		int turnLeftPoints = 0;
	public:
		//Instruction(Robot& robot, void(*_operation)(), bool(*_stopCondition)(), int _minTime, int _maxTime);
		Instruction(Robot& _robot, function<void (int& turnLeftPoints)> _operation, function<bool ()> _stopCondition, int _minTime, int _maxTime, int _extraTime = 0);
		//void (*operation)();
		//bool (*stopCondition)();
		function<void ()> operation;
		function<bool ()> stopCondition;
		int minTime;
		int maxTime;
		int extraTime;
};
#endif
