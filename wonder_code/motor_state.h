/***************************************
 * File: motor_state.h
 * Description: This file defines a common structure
 * 	for communicating the state that the motors should
 * 	be in.
 * 	It contains information about the speed and 
 * 	running of each motor
 ****************************************/
//Motor Controller Interface Macros
//#define MTR_CTRLR_LEFT_ENABLE (1<<PD0)
//#define MTR_CTRLR_RIGHT_ENABLE (1<<PD1)
//#define MTR_CTRLR_LEFT_DIR (1<<PD2)
//#define MTR_CTRLR_RIGHT_DIR (1<<PD3)

/* PORTD's pins 2 and 3 are tied together and do not function!*/
//Didn't use enums because they're ints (different sizes between architectures and a lot to transmit)
unsigned char NEW_MTR_STATE_OP = 1;
unsigned char LEFT_MOTOR_PIN = 4, RIGHT_MOTOR_PIN = 6;
unsigned char MOTOR_DIS = 1, MOTOR_EN = 0;
unsigned char BACKWARD_DIR = 0, FORWARD_DIR = 1;

struct motor_state {
	unsigned char motor_name;
	unsigned char enabled;
	unsigned char dir;
	unsigned char power_percentage;
};

struct total_state {
	unsigned char motor_cnt;
	struct motor_state* motors;
};
