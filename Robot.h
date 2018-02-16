#include <robot_link.h>
#include <stopwatch.h>
#include "StopwatchWithPause.h"
#include <queue>
#include <fstream>
#include <iostream>

#ifndef ROBOT_H
#define ROBOT_H
class Robot{
	private:
		int doubleToUInt(double number);
        double limitSpeed(double speed);
        int setBitValue(int number, int bit, bool value);
		robot_link rlink;
		int leftSpeed;
		int rightSpeed;
		int lineSensors[4]; //left sensor first, rightmost last
		int lineSensorsLastTriggered[2][4];
		stopwatch watch;
		int pinValues[8];
		int distanceSensorVoltage;
		bool getBitValueAsBoolean(int value, int bit);
		int LDRFront;
		int LDREntriesLength;
		int LDREntriesTotal;
	public:
		Robot();
        static const int NUMBER_VALUES_STORED = 10;
        int integralCount = 0;
        int getDistanceSensorVoltage();
        int previousDistanceIndex;
        int previousDistanceValues[NUMBER_VALUES_STORED];
        int previousLDRIndex = 0;
		void initialiseRobotLink();
		void initialiseSensors();
		int sendTestInstruction();
		void printErrors();
		void setLeftMotor(double speed);
		void setRightMotor(double speed);
        void setMechanismMotor(double speed);
		void setMotors(double leftSpeed, double rightSpeed);
		void stopMotors();
		void updateLineSensors();
		bool checkLineSensorsMatch(int sensorState[]);
		bool checkLineSensorsMatchFuzzy(int sensorState[], int interval);
		void activateHarvestMechanism();
		void deactivateHarvestMechanism(bool force = false);
		void updateDistanceSensor();
        bool getStartButtonPressed();
        bool getStartButtonPressedWithUpdate();
        void readPinValue(int port);
        void awaitStartButtonPress();
        void turnMechanism();
        void updateLDRFront(bool udpateMean = true);
        void updateLDRBack(bool udpateMean = true);
        int getLDRFront();
        void clearMeanLDR();
        double meanLDR;
        int brassicaStartTime;
        int brassicaEndTime;
        int brassicaMaxValue;
        int brassicaMinValue;
        queue<int> brassicaHarvestTimes;
        StopwatchWithPause harvestTimer;
        void harvestBrassica();
        ofstream myfile;
        ofstream myfile2;
        void resetIntegral();
};
#endif
