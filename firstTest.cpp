#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <robot_link.h>
#define ROBOT_NUM  16                         // The id number (see below)
#include <robot_delay.h>
#include <stopwatch.h>
robot_link  rlink;                            // datatype for the robot link
int main ()
{
	int   val;                                // data from microprocessor
	#ifdef __arm__
		if (!rlink.initialise ("127.0.0.1")) {          // setup for local hardware
	#else
	   if (!rlink.initialise (ROBOT_NUM)) { // setup the link
	#endif
		cout << "Cannot initialise link" << endl;
		rlink.print_errs("    ");
		return -1;
	}
	val = rlink.request (TEST_INSTRUCTION);   // send test instruction
		if (val == TEST_INSTRUCTION_RESULT) {     // check result
		cout << "Test passed" << endl;
		
		//rlink.command (MOTOR_1_GO, 127); //RED = 1
		//rlink.command (MOTOR_2_GO, 127); //GREEN = 2
		//delay(10000);
		
		//38.5ms for writing
		//5ms for reading
		//2ms reading on robot
		
		
		//int value = 255;
		//stopwatch watch;
		//watch.start();
		//for(int i=0; i< 100; i ++){
			//rlink.command(WRITE_PORT_5, value);
		//}
		//int time = watch.read();
		//watch.stop();
		
		rlink.command(MOTOR_1_GO, 192);
		rlink.command(MOTOR_2_GO, 64);
		delay(10000);
		
		rlink.command(MOTOR_1_GO, 64);
		rlink.command(MOTOR_2_GO, 192);
		delay(10000);
		
		//rlink.command(MOTOR_1_GO, 64);
		//delay(300);
		//rlink.command(MOTOR_1_GO, 0);
		//rlink.command(MOTOR_2_GO, 192);
		//delay(300);
		//rlink.command(MOTOR_2_GO, 0);
		//rlink.command(MOTOR_1_GO, 64);
		//delay(300);
		//rlink.command(MOTOR_1_GO, 0);
		//rlink.command(MOTOR_2_GO, 192);
		//delay(300);
		
		//for(int i =0; i < 10; i++){
		//rlink.command(MOTOR_2_GO, 0);
		//rlink.command(MOTOR_1_GO, 64);
		//delay(300);
		//rlink.command(MOTOR_1_GO, 0);
		//rlink.command(MOTOR_2_GO, 192);
		//delay(300);
	//}
		
		return 0;                             // all OK, finish
	}
	else if (val == REQUEST_ERROR) {
		cout << "Fatal errors on link:" << endl;
		rlink.print_errs();
	}
	else
		cout << "Test failed (bad value returned)" << endl;
	return -1;                                // error, finish

}
