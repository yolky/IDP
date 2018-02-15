#include "StopwatchWithPause.h"

StopwatchWithPause::StopwatchWithPause(){
    totalTime = 0;
}

int StopwatchWithPause::read(){
    return running? totalTime + watch.read() : totalTime;
}

void StopwatchWithPause::pause(){
    if(running){
        totalTime += watch.read();
        watch.stop();
        running = false;
    }
}

void StopwatchWithPause::start(){
    if(!running){
        watch.start();
        running = true;
    }
}


void StopwatchWithPause::stop(){
    if(running){
        totalTime = 0;
        watch.stop();
        running = false;
    }
}
