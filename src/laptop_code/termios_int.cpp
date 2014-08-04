//Feature Test Macros
#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE 600
#define _BSD_SOURCE 100000L

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#include "../wonder_code/motor_state.h"

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;

bool operator == (struct termios& config1, struct termios& config2)
{
	if (config1.c_iflag == config2.c_iflag
		&& config1.c_oflag == config2.c_oflag
		&& config1.c_cflag == config2.c_cflag
		&& config1.c_lflag == config2.c_lflag){
		return true;
	}

	return false;
}	

class MotorTranslator{
public:
	MotorTranslator();
	MotorTranslator(int num_motors, int conn_fd);

	void push_joy_to_motors();
	void set_new_motor_state(const sensor_msgs::Joy::ConstPtr& msg);

private:
	int motor_fd;
	int motor_cnt;
	bool has_new_motor_state;
	sensor_msgs::Joy motor_state;
	struct motor_state* motors;

	double get_angle_of_stick(vector<float> axes);
	double get_magnitude_of_stick(vector<float> axes);	
	void mk_motor_msg(double theta, double mag);
	void joy_to_motor_dir_and_speed(struct motor_state* motor, double power);
	double get_opposite_motor_power(double theta);
};

string get_device_loc(int argc, char** argv);
int open_conn(string& device_location);
void configure_connection(int conn_fd, struct termios& config);

void transmit_motor_state(struct motor_state *motors, int num_motors, int conn_fd);
void write_op(int motor_fd, unsigned char op);
void write_data_to_motors(int motor_fd, char* start_of_data, char* end_of_data);
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);

void configure_motor(struct motor_state* motor);
void mk_manual_motor_msg(struct motor_state* motors, int num_motors);

MotorTranslator* motor_translator;

int main(int argc, char** argv){
	ros::init(argc, argv, "motor_controller");
	ros::NodeHandle n;
	string device_location = get_device_loc(argc, argv);
	int conn_fd = open_conn(device_location);

	struct termios conn_properties;
	configure_connection(conn_fd, conn_properties);
	
	/*while (1){
		mk_manual_motor_msg(motors, 2);
		transmit_motor_state(motors, 2, conn_fd);
	}*/

	motor_translator = new MotorTranslator(2, conn_fd);
	ros::Subscriber joy_subsciber = n.subscribe<sensor_msgs::Joy>("joy", 10, joy_callback);
	ros::AsyncSpinner spinny(1);
	spinny.start();

	ros::Duration peace(0.2);
	while(ros::ok()){
		motor_translator->push_joy_to_motors();
		peace.sleep();
	}
}

string get_device_loc(int argc, char** argv)
{
	string device_location;
	
	if (argc == 1){
		cout << "No device name given via command line; setting to '/dev/ttyUSB0'" << endl;
		device_location = "/dev/ttyUSB0";
	} else {
		device_location = argv[1];
	}

	return device_location;
}

int open_conn(string& device_location)
{
	int conn_fd = open(device_location.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (conn_fd == -1){
		perror("Could not open connection: ");
		exit(1);
	}

	if (!isatty(conn_fd)){
		cout << "We're seeing that the device is not a TTY?" << endl;
		exit(1);
	} else {
		cout << "Device connection established." << endl;
	}

	return conn_fd;
}


void configure_connection(int conn_fd, struct termios& config)
{
	if (tcgetattr(conn_fd, &config) < 0){
		cout << "Could not get initial connection configuration." << endl;
		exit(1);
	}

	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag = 0;
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	//8 bit character, no parity
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 0;

	if (cfsetspeed(&config, B9600) == -1){
		perror("Could not set baud rate: ");
		exit(1);
	}

	tcsetattr(conn_fd, TCSANOW, &config);
	struct termios temp;
	tcgetattr(conn_fd, &temp);
	if (temp == config){
		cout << "Success, connection settings configured." << endl;
	} else {
		cout << "Connection configuration failed!" << endl;
		exit(1);
	}
}

void transmit_motor_state(struct motor_state *motors, int num_motors, int conn_fd)
{
	write_op(conn_fd, NEW_MTR_STATE_OP);

	char* end_motor_data = (char*) motors + (sizeof(motor_state) * num_motors);
	write_data_to_motors(conn_fd, (char*) motors, end_motor_data);
}

void write_op(int motor_fd, unsigned char op)
{
	int ret_val = write(motor_fd, &op, sizeof(op));
	tcdrain(motor_fd);
	if (ret_val == -1){
		perror("Could not write motor op to connection: ");
		exit(1);

	} else {
		cout << "Wrote: " << ret_val << " bytes for op code." << endl;
	}
}

//Writes contiguous data such as an array
void write_data_to_motors(int motor_fd, char* start_of_data, char* end_of_data)
{
	if (abs(end_of_data - start_of_data) > 2){
		cout << "write_data_to_motors() sends data one byte at a time, could be optimized." << endl;
	}

	int ret_val;
	while(start_of_data != end_of_data){
		ret_val = write(motor_fd, start_of_data, 1);
		if (ret_val == -1){
			perror("Could not write motor states to connection: ");
		}

		start_of_data++;
	}
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
	cout << "Got joy message!" << endl;
	ROS_INFO("Got joy message =)");
	motor_translator->set_new_motor_state(msg);
}

MotorTranslator::MotorTranslator()
{
	cout << "ERROR! Default constructor called for MotorTranslator, not possible to reconcile." << endl;
	exit(1);
}

MotorTranslator::MotorTranslator(int num_motors, int conn_fd)
{
	motor_cnt = num_motors;
	motors = new struct motor_state[motor_cnt];
	has_new_motor_state = false;

	motor_fd = conn_fd;
}

void MotorTranslator::set_new_motor_state(const sensor_msgs::Joy::ConstPtr& msg)
{
	motor_state = *msg;
	has_new_motor_state = true;
}

void MotorTranslator::push_joy_to_motors(){
	if (!has_new_motor_state){
		return;

	} else {
		has_new_motor_state = false;
	}

	double theta_rad = get_angle_of_stick(motor_state.axes);
	double mag = get_magnitude_of_stick(motor_state.axes);

	mk_motor_msg(theta_rad, mag);

	transmit_motor_state(motors, motor_cnt, motor_fd);
}

double MotorTranslator::get_angle_of_stick(vector<float> axes)
{
	double theta;
	if (axes[0] == 0){
		if (axes[1] >= 0){
			theta = M_PI / 2;
		} else {
			theta = (3 * M_PI) / 2;
		}	
	} else {
		theta = atan(axes[1] / axes[0]);
		cout << "Original theta" << theta << endl;
		if (axes[0] < 0) {
			theta = M_PI - theta;
		}
	}

	return theta;
}

double MotorTranslator::get_magnitude_of_stick(vector<float> axes)
{
	return sqrt(pow(axes[0], 2) + pow(axes[1], 2));
}

void MotorTranslator::mk_motor_msg(double theta, double mag)
{
	if (motor_cnt != 2){
		cout << "mk_motor_msg was not build for other than 2 motors. Preserving state." << endl;
		return;
	}

	set_motor_left(motors);
	set_motor_right(motors + 1);
	set_motor_enable(motors, 1);
	set_motor_enable(motors + 1, 1);

	/*double l_power = sin(M_PI - 0.5*theta);
	double r_power = sin(0.5 * theta);
	l_power >= r_power ? (mag *= (1 / l_power)) : (mag *= (1 / r_power));
	l_power *= mag;
	r_power *= mag;
*/
	double l_power, r_power;
	if (theta > M_PI / 2){
		r_power = 100 * mag;
		l_power = get_opposite_motor_power(theta) * mag;

	} else {
		l_power = 100 * mag;
		r_power = get_opposite_motor_power(theta) * mag;
	}

	if (theta > M_PI){
		double temp = l_power;
		l_power = -r_power;
		r_power = -temp;
	}

	cout << "l_power: " << l_power << " r_power: " << r_power << endl;

	joy_to_motor_dir_and_speed(motors, l_power);
	joy_to_motor_dir_and_speed(motors + 1, r_power);

	cout << "Lspeed: " << (int) get_motor_speed(motors) 
			<< " Rspeed: " << (int) get_motor_speed(motors + 1) << endl;
}

double MotorTranslator::get_opposite_motor_power(double theta)
{
	cout << "Theta: " << theta << " res: " << (100 - 200 * fabs(cos(theta))) << endl;
	return (100 - 200 * fabs(cos(theta)));
}

void MotorTranslator::joy_to_motor_dir_and_speed(struct motor_state* motor, double power)
{
	if (power < 0){
		set_motor_dir(motor, 0);
	} else { 
		set_motor_dir(motor, 1);
	}

	set_motor_speed(motor, fabs(power));
}

void all_motors(double magnitude, struct motor_state* motors, int num_motors, unsigned char dir)
{
	for (int i = 0; i < num_motors; ++i){
		set_motor_enable(motors + i, 1);
		set_motor_dir(motors + i, dir);
		set_motor_speed(motors + i, 100 * magnitude);
	}
}

void mk_manual_motor_msg(struct motor_state* motors, int num_motors)
{
	
	set_motor_left(motors + 0);
	cout << "Left Motor: " << endl;
	configure_motor(motors + 0);

	set_motor_right(motors + 1);
	cout << "Right Motor: " << endl;
	configure_motor(motors + 1);
}

void configure_motor(struct motor_state* motor)
{
	string dir;
	string en;
	string speed;

	cout << "\tEnable (y/else): ";
	cin >> en;
	if (en == "y" || en == "Y"){
		set_motor_enable(motor, 1);
		cout << "\tDirection (f/b): ";
		cin >> dir;
		if (dir == "f" || dir == "F"){
			set_motor_dir(motor, 1);
		} else {
			set_motor_dir(motor, 0);
		}
	} else {
		set_motor_enable(motor, 0);
	}

}