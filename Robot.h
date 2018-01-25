#include <robot_link.h>

#ifndef ROBOT_H
#define ROBOT_H
class Robot{
	private:
		int doubleToUInt(double number);
		robot_link rlink;
	public:
		Robot();
		void initialiseRobotLink();
		int sendTestInstruction();
		void printErrors();
		void setLeftMotor(double speed);
		void setRightMotor(double speed);
		void setMotors(double leftSpeed, double rightSpeed);
		void stopMotors();
		
};
#endif
