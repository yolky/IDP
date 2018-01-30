#include <robot_link.h>
#include <stopwatch.h>

#ifndef ROBOT_H
#define ROBOT_H
class Robot{
	private:
		int doubleToUInt(double number);
        double limitSpeed(double speed);
		robot_link rlink;
		int leftSpeed;
		int rightSpeed;
		int lineSensors[4]; //left sensor first, rightmost last
		int lineSensorsLastTriggered[2][4];
		stopwatch watch;
	public:
		Robot();
		void initialiseRobotLink();
		void initialiseSensors();
		int sendTestInstruction();
		void printErrors();
		void setLeftMotor(double speed);
		void setRightMotor(double speed);
		void setMotors(double leftSpeed, double rightSpeed);
		void stopMotors();
		void updateLineSensors();
		bool checkLineSensorsMatch(int sensorState[]);
		bool checkLineSensorsMatchFuzzy(int sensorState[], int interval);
};
#endif
