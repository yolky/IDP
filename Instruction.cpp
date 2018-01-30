#include "Robot.h"
#include "Instruction.h"

Instruction::Instruction(Robot& _robot, function<void (int& turnLeftPoints)> _operation, function<bool ()> _stopCondition, int _minTime, int _maxTime, int _extraTime){
	robot = _robot;
	operation = bind(_operation, turnLeftPoints);
	stopCondition = _stopCondition;
	minTime = _minTime;
	maxTime = _maxTime;
	extraTime = _extraTime;
}
