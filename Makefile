# Automatically generated Makefile
main : main.cpp  Robot.h Instruction.h InstructionHandler.h
	 g++ -ansi -Wall -g -I/export/teach/1BRobot -L/export/teach/1BRobot main.cpp -o main -lrobot
