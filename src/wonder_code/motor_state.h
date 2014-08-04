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
const unsigned char NEW_MTR_STATE_OP = 1;
const unsigned char LEFT_MOTOR = 128, RIGHT_MOTOR = 0;
const unsigned char MOTOR_DIS = 32, MOTOR_EN = 0;
const unsigned char BACKWARD_DIR = 0, FORWARD_DIR = 16;
const unsigned char SPEED_MASK = 1 + 2 + 4 + 8;
//Power is between 0 and 10;

#define IS_FORWARD(n) (n & FORWARD_DIR)

struct motor_state {
	unsigned char motor_state;
};

/*struct motor_state {
	unsigned char motor_name;
	unsigned char enabled;
	unsigned char dir;
	unsigned char power_percentage;
};*/

struct total_state {
	unsigned char motor_cnt;
	struct motor_state* motors;
};

unsigned char motor_is_enabled(struct motor_state* motor)
{
	if (motor->motor_state & MOTOR_DIS){
		return 0;
	}

	return 1;
}

unsigned char motor_is_forward(struct motor_state* motor)
{
	if (motor->motor_state & FORWARD_DIR){
		return 1;
	}

	return 0;
}

unsigned char get_motor_speed(struct motor_state* motor)
{
	return (motor->motor_state & SPEED_MASK);
}

//1 for enabled, 0 for disabled.
void set_motor_enable(struct motor_state* motor, unsigned char en)
{
	if (en){
		motor->motor_state &= ~MOTOR_DIS;
	} else {
		motor->motor_state |= MOTOR_DIS;
	}
}

void set_motor_dir(struct motor_state* motor, unsigned char dir_forward)
{
	if (dir_forward){
		motor->motor_state |= FORWARD_DIR;
	} else {
		motor->motor_state &= ~FORWARD_DIR;
	}
}

void set_motor_speed(struct motor_state* motor, double power_percentage)
{
	if (power_percentage > 100){
		power_percentage = 100;
	}

	if (power_percentage < 0){
		power_percentage = 0;
	}
	motor->motor_state &= ~SPEED_MASK;
	motor->motor_state += (unsigned char)(15 * (power_percentage / 100));
}

void set_motor_left(struct motor_state* motor)
{
	motor->motor_state |= LEFT_MOTOR;
}

void set_motor_right(struct motor_state* motor)
{
	motor->motor_state &= ~LEFT_MOTOR;
}
