#include <robot_link.h>

#ifndef ROBOT_H
#define ROBOT_H
class Robot{
	private:
		int doubleToUInt(double number);
		robot_link rlink;
		int leftSpeed;
		int rightSpeed;
		bool lineSensors[4]; //left sensor first, rightmost last
	public:
		Robot();
		void initialiseRobotLink();
		void initiliseSensors();
		int sendTestInstruction();
		void printErrors();
		void setLeftMotor(double speed);
		void setRightMotor(double speed);
		void setMotors(double leftSpeed, double rightSpeed);
		void stopMotors();
		void updateLineSensors();
		bool Robot : checkLineSensorsMatch(int sensorState[]);
};
#endif
