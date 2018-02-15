#include "Robot.h"
#include "Instruction.h"

Instruction::Instruction(Robot& _robot, function<void ()> _operation, int _minTime, int _maxTime, int _extraTime):
    robot(_robot)
{
	operation = _operation;
	minTime = _minTime;
	maxTime = _maxTime;
	extraTime = _extraTime;
}


Instruction::Instruction(Robot& _robot, function<void (int previousValues[], int &previousValuesLength, int& previousValueIndex)> _operation, int _minTime, int _maxTime, int _extraTime):
	robot(_robot)
{
	operation = bind(_operation, robot.previousDistanceValues, 5, robot.previousDistanceIndex);
	minTime = _minTime;
	maxTime = _maxTime;
	extraTime = _extraTime;
}

void Instruction::setPreInstruction(function<void ()>& _preInstruction){
    preInstruction = _preInstruction;
    hasPreInstruction = true;
}

void Instruction::setPostInstruction(function<void ()>& _postInstruction){
    postInstruction = _postInstruction;
    hasPostInstruction = true;
}

void Instruction::startRunning(){
    watch.start();
}

void Instruction::stopRunning(){
    watch.stop();
}

int Instruction::getRunningTime(){
    return watch.read();
}

bool Instruction::evaluateStopConditions(){
    cout << 'here' << endl;
    for(int i =0; i<numberStopConditions; i++){
        cout << "dead" << endl;
        if((*stopConditions[i])()){
            selectedNextInstruction = i;
            return true;
        }
    }
    return false;
}

void Instruction::setStopCondition(function<bool()>& stopCondition){
    stopConditions[0] = &stopCondition;
    numberStopConditions = 1;
}

void Instruction::setStopConditions(function<bool()>* _stopConditions[], int _numberStopConditions){
    numberStopConditions = _numberStopConditions;
    for(int i =0; i < numberStopConditions; i++){
        stopConditions[i] = _stopConditions[i];
    }
}

void Instruction::setNextInstruction(Instruction& instruction){
    isEndState = false;
    nextInstructions[0] = &instruction;
}

void Instruction::setNextInstructions(Instruction* instructions[], int numberInstructions){
    isEndState = false;
    for(int i =0; i < numberInstructions; i++){
        nextInstructions[i] = instructions[i];
    }
}

Instruction& Instruction::getNextInstruction(){
    return *nextInstructions[selectedNextInstruction];
}
