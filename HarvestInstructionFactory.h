#include "globals.cpp"

#ifndef HARVESTINSTRUCTIONFACTORY_H_INCLUDED
#define HARVESTINSTRUCTIONFACTORY_H_INCLUDED
class HarvestInstructionFactory{
	private:
        Robot& robot;
        function<bool()> beforeStripStop;
	public:
		HarvestInstructionFactory(Robot& _robot);
		Instruction beginHarvest;
        Instruction preBrassica;
        Instruction overCauliflower;
        Instruction overCabbage;
        Instruction activateMechanism;
};


#endif // HARVESTINSTRUCTIONFACTORY_H_INCLUDED
