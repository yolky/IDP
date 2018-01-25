#include "Robot.h"
#include "Instruction.h"

Instruction::Instruction(Robot& _robot, void(*_operation)(), bool(*_stopCondition)(), int _minTime, int _maxTime){
	robot = _robot;
	operation = _operation;
	stopCondition = _stopCondition;
	minTime = _minTime;
	maxTime = _maxTime;
}
