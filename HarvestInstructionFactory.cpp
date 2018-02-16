#include "globals.cpp"
#include "Instruction.h"

#include <functional>
using namespace std;

void flatForwardHarvest(bool harvestCauliflowers);
function<void()> overBrassicaOperation(Robot& robot, double speed, double kP);
function<void()> preBrassicaOperation(Robot& robot, double speed, double kP);
void checkForHarvestTimesAndRunMechanism(Robot& robot);

function<void()> overBrassicaOperation(Robot& robot, double speed, double kP){
    return [&, speed, kP](){
        checkForHarvestTimesAndRunMechanism(robot);
        followLineOperation(robot, speed, kP)();
        robot.brassicaMaxValue = max(robot.brassicaMaxValue, robot.getLDRFront());
        robot.brassicaMinValue = min(robot.brassicaMinValue, robot.getLDRFront());
    };
}

function<void()> preBrassicaOperation(Robot& robot, double speed, double kP){
    return [&, speed, kP](){
        checkForHarvestTimesAndRunMechanism(robot);
        followLineOperation(robot, speed, kP)();
    };
}

void checkForHarvestTimesAndRunMechanism(Robot& robot){
    if(robot.brassicaHarvestTimes.front()!= 0 && (robot.harvestTimer.read() > robot.brassicaHarvestTimes.front())){
        robot.harvestTimer.pause();
        robot.stopMotors();
        delay(200);
        robot.brassicaHarvestTimes.pop();
        cout << "harvesting!!" << endl;
        robot.harvestBrassica();
        robot.harvestTimer.start();
    }
};

void flatForwardHarvest(bool harvestCauliflowers){
    function<void()> clearMeanFunction = [&robot](){robot.clearMeanLDR();};
    function<bool()> beforeStripStop = aboveMeanStopCondition(robot, -10, true, false);
    function<bool()> beforeStripStopCabbage = [&robot](){
        if(aboveMeanStopCondition(robot, 9, true, true)()){
            robot.meanLDR -= 7;
            robot.harvestTimer.start();
            robot.harvestTimer.setTime(-400);
            return true;
        }
        return false;
    };

    function<bool()> beforeStripStopCauliflower = [&robot](){
        if(aboveMeanStopCondition(robot, -20, true, true)()){
            robot.meanLDR -= 7;
            robot.harvestTimer.start();
            robot.harvestTimer.setTime(-400);
            return true;
        }
        return false;
    };

    function<bool()>* beforeStripStopConditions[] = {&beforeStripStopCauliflower, &beforeStripStop, &beforeStripStopCabbage};

    function<bool()> preBrassicaStopCauliflower = aboveMeanStopCondition(robot, -9, true, false);
    function<bool()> preBrassicaStopCabbage = aboveMeanStopCondition(robot, 18, true, true);
    function<bool()> preBrassicaStopToHarvest = [&](){
        return (robot.brassicaHarvestTimes.front()!= 0 && (robot.harvestTimer.read() > robot.brassicaHarvestTimes.front()));
    };

    function<bool()> tJuctionMinTime = [&robot, &STOP_CONDITION_T_JUNCTION](){
        return((robot.harvestTimer.read() > 3000) && STOP_CONDITION_T_JUNCTION());
    };

    function<bool()>* preBrassicaStopConditions[] = {&preBrassicaStopCauliflower, &preBrassicaStopCabbage, &tJuctionMinTime};
    function<void()> preBrassicaPreFunction = [&robot](){
        robot.harvestTimer.start();
    };

    function<bool()> overCauliflowerStopCondition = aboveMeanStopCondition(robot, -7, false, true);;
    function<void()> recordStartTime = [&robot](){
        robot.brassicaStartTime = robot.harvestTimer.read();
        robot.brassicaMaxValue = 0;
        robot.brassicaMinValue = 999;
    };
    function<void()> analyseBrassicaValues = [&robot, harvestCauliflowers](){
        robot.brassicaEndTime = robot.harvestTimer.read();
        int brassicaWidth = robot.brassicaEndTime - robot.brassicaStartTime;
        int brassicaMax = robot.brassicaMaxValue - robot.meanLDR;
        int brassicaMin = robot.brassicaMinValue - robot.meanLDR;
        cout<< " max: " << brassicaMax << " min: " << brassicaMin << " " << brassicaWidth << endl;



        cout << "wid " << brassicaWidth << "  height  " << brassicaMax << endl;

        if(brassicaWidth > 300){
            robot.clearMeanLDR();
            if(brassicaMin < -12){

                cout << "LARGE CAULIFLOWER" << endl;
                if(harvestCauliflowers){
                    robot.brassicaHarvestTimes.push(robot.brassicaStartTime + 1100);
                }
            }

            if(brassicaMax > 40){
                cout << "LARGE CABBAGE" << endl;
                if(!harvestCauliflowers){
                    robot.brassicaHarvestTimes.push(robot.brassicaStartTime + 1500);
                }
            }
            cout << "wid " << brassicaWidth << "  height  " << brassicaMax << endl;
        }
    };


    function<bool()> overCabbageStopCondition = aboveMeanStopCondition(robot, 2, false, false);
    function<void()> startHarvestTimer = [&robot](){
        robot.harvestTimer.start();
    };

    function<void()> endInstructionPreInstruction = [&robot](){
        robot.stopMotors();
        robot.harvestBrassica();
        robot.harvestTimer.stop();
        queue<int> empty;
        swap( robot.brassicaHarvestTimes, empty);
    };



    Instruction shortReverse(robot, followLineOperation(robot, -0.5, -0.2), 400, 3000);

    Instruction beginHarvest(robot, followLineOperation(robot, 0.3, 0.2), 220, 3000);
    Instruction preBrassica(robot, preBrassicaOperation(robot, 0.3, 0.2), 3, 10000);
    Instruction overCauliflower(robot, overBrassicaOperation(robot, 0.3, 0.2),30, 1000);
    Instruction overCabbage(robot, overBrassicaOperation(robot, 0.3, 0.2),200, 1000);

    Instruction endInstruction(robot, doNothingOperation(),0, 1000);

    Instruction* preBrassicaNextInstructions[] = {&overCauliflower, &overCabbage, &endInstruction};
    Instruction* beginHarvestNextInstructions[] = {&overCauliflower, &preBrassica, &overCabbage};

    shortReverse.setStopCondition(alwaysStopCondition);
    shortReverse.setNextInstruction(beginHarvest);

    beginHarvest.setPreInstruction(clearMeanFunction);
    beginHarvest.setStopConditions(beforeStripStopConditions, 3);
    beginHarvest.setNextInstructions(beginHarvestNextInstructions, 3);
    beginHarvest.setPostInstruction(clearMeanFunction);

    preBrassica.setStopConditions(preBrassicaStopConditions, 3);
    preBrassica.setNextInstructions(preBrassicaNextInstructions, 3);
    preBrassica.setPreInstruction(preBrassicaPreFunction);

    overCauliflower.setStopCondition(overCauliflowerStopCondition);
    overCauliflower.setNextInstruction(preBrassica);
    overCauliflower.setPreInstruction(recordStartTime);
    overCauliflower.setPostInstruction(analyseBrassicaValues);

    overCabbage.setStopCondition(overCabbageStopCondition);
    overCabbage.setNextInstruction(preBrassica);
    overCabbage.setPreInstruction(recordStartTime);
    overCabbage.setPostInstruction(analyseBrassicaValues);
    overCabbage.setDefaultNextInstructionIndex(0);

    endInstruction.setStopCondition(alwaysStopCondition);
    endInstruction.setPreInstruction(endInstructionPreInstruction);

    instructionHandler.runInstruction(beginHarvest);
}

