#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>
#include "Robot.h"
#include "Robot.cpp"
#include "Instruction.h"
#include "Instruction.cpp"
#include "InstructionHandler.h"
#include "InstructionHandler.cpp"

void bullShit();
                          // datatype for the robot link
int main ()
{
	Robot robot;
	
	Instruction testInstruction(robot, bullShit);
	testInstruction.operation();
	Instruction instructions [1] = {testInstruction};
	
	InstructionHandler instructionHandler;
	instructionHandler.runInstructions(instructions);
	
	int val = robot.sendTestInstruction();  // send test instruction
	if (val == TEST_INSTRUCTION_RESULT) {     // check result
		cout << "Test passed" << endl;
		
		robot.setMotors(.5, .5);
		delay(5000);

		return 0;                             // all OK, finish
	}
	else if (val == REQUEST_ERROR) {
		cout << "Fatal errors on link:" << endl;
	}
	else
		cout << "Test failed (bad value returned)" << endl;
	return -1;                                // error, finish

}

void bullShit(){
	cout<<"something belse" << endl;
}
