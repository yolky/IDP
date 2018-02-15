#include <stopwatch.h>

#ifndef STOPWATCHWITHPAUSE_H
#define STOPWATCHWITHPAUSE_H
class StopwatchWithPause{
	private:
        int totalTime;
        stopwatch watch;
        bool running = false;
	public:
        StopwatchWithPause();
        int read();
        void pause();
        void start();
        void stop();
};
#endif
