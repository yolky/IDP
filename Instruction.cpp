#include "Robot.h"
#include "Instruction.h"

Instruction::Instruction(Robot& _robot, void(*_operation)()){
	robot = _robot;
	operation = _operation;
}
