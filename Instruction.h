#include "Robot.h"
#include <functional>
#include <stopwatch.h>

#ifndef INSTRUCTION_H
#define INSTRUCTION_H
class Instruction{
	private:
		int selectedNextInstruction = 0;
		vector<Instruction> nextInstructionsTest;
		int numberStopConditions;
		stopwatch watch;
	public:
        Robot& robot;
        		Instruction* nextInstructions[12];
            		function<bool()>* stopConditions[12];

		//Instruction(Robot& robot, void(*_operation)(), bool(*_stopCondition)(), int _minTime, int _maxTime);
		Instruction(Robot& _robot, function<void ()> _operation, int _minTime, int _maxTime, int _extraTime = 0);
        Instruction(Robot& _robot, function<void (int previousValues[], int& previousValuesLength, int& previousValueIndex)> _operation, int _minTime, int _maxTime, int _extraTime = 0);
		//void (*operation)();
		//bool (*stopCondition)();
		function<void ()> operation;
		function<void ()> preInstruction;
        function<void ()> postInstruction;
		bool evaluateStopConditions();
		void setStopCondition(function<bool()>& stopCondition);
		void setStopConditions(function<bool()>* _stopConditions[], int _numberStopConditions);
		void setNextInstruction(Instruction& instruction);
		void setNextInstructions(Instruction* instructions[], int numberInstructions);
		Instruction& getNextInstruction();
		int minTime;
		int maxTime;
		int extraTime;
		bool isEndState = true;
		void startRunning();
		void stopRunning();
		int getRunningTime();
		bool hasPreInstruction = false;
		bool hasPostInstruction = false;
		void setPreInstruction(function<void ()>& _preInstruction);
		void setPostInstruction(function<void ()>& _postInstruction);
};
#endif
