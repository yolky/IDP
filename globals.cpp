
function<bool()> getLineSensorStopCondition(Robot& robot, int sensorState[]);
function<bool()> getLineSensorStopConditionFuzzy(Robot& robot, int sensorState[], int interval);
function<bool()> alwaysStop();

Robot robot;

int SENSOR_T_JUNCTION [4] =  {1, -1, -1, 1 };
int SENSOR_TEST[4] = {-1,1,-1,-1};
int SENSOR_TEST2[4] = {-1,-1,1,-1};
int SENSOR_FAR_RIGHT[4] = {-1, -1, -1, 1};
int SENSOR_FAR_LEFT[4] = {1, -1, -1, -1};


function <bool()> STOP_CONDITION_T_JUNCTION = getLineSensorStopConditionFuzzy(robot, SENSOR_T_JUNCTION,250);
function <bool()> STOP_CONDITION_LEFT_TURN = getLineSensorStopCondition(robot, SENSOR_TEST);
function <bool()> STOP_CONDITION_RIGHT_TURN = getLineSensorStopCondition(robot, SENSOR_TEST2);
function <bool()> STOP_CONDITION_FAR_RIGHT = getLineSensorStopCondition(robot, SENSOR_FAR_RIGHT);
function<bool()> alwaysStopCondition = alwaysStop();

function<bool ()> getLineSensorStopConditionFuzzy(Robot& robot, int sensorState[], int interval) {
	return [&, sensorState, interval]() {
		robot.updateLineSensors();
        return robot.checkLineSensorsMatchFuzzy(sensorState, interval);
	};
}

function<bool ()> getLineSensorStopCondition(Robot& robot, int sensorState[]) {
	return [&, sensorState]() {
        return robot.checkLineSensorsMatch(sensorState);
	};
}

function<bool()> alwaysStop(){
    return [](){
        return true;
    };
}
