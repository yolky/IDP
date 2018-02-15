#include "globals.cpp"

#ifndef HARVESTINSTRUCTIONFACTORY_H_INCLUDED
#define HARVESTINSTRUCTIONFACTORY_H_INCLUDED
class HarvestInstructionFactory{
	private:
        Robot& robot;
	public:
		HarvestInstructionFactory(Robot& _robot);
		Instruction beginHarvest(robot, followLineOperation(robot, 0.3, 0.2), 100, 0);
        Instruction preBrassica(robot, followLineOperation(robot, 0.3, 0.2), 10, 10000);
        Instruction overCauliflower(robot, overBrassicaOperation(robot, 0.3, 0.2),30, 10000);
        Instruction overCabbage(robot, overBrassicaOperation(robot, 0.3, 0.2),200, 10000);
        Instruction activateMechanism(robot, doNothingOperation(robot), 20,10000);
};


#endif // HARVESTINSTRUCTIONFACTORY_H_INCLUDED
