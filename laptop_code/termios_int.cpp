//Feature Test Macros
#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE 600
#define _BSD_SOURCE 100000L

#include <iostream>
#include <string>

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

string get_device_loc(int argc, char** argv);
int open_conn(string& device_location);
void configure_connection(int conn_fd, struct termios& config);

void transmit_motor_state(struct motor_state *motors, int num_motors, int conn_fd);

int main(int argc, char** argv){
	struct termios conn_properties;
	string device_location = get_device_loc(argc, argv);
	int conn_fd = open_conn(device_location);

	configure_connection(conn_fd, conn_properties);

	struct motor_state left_motor = {LEFT_MOTOR_PIN, MOTOR_EN, FORWARD_DIR, 50};
	//struct motor_state left_motor = {1, 2, 3, 4};
	struct motor_state right_motor = {RIGHT_MOTOR_PIN, MOTOR_EN, BACKWARD_DIR, 50};
	//struct motor_state right_motor = {5, 6, 7, 8};
	struct motor_state motors[2];
	motors[0] = left_motor; motors[1] = right_motor;

//	transmit_motor_state(motors, 2, conn_fd);

	cout << "Sizeof motor_state struct" << sizeof(struct motor_state) << endl;

	char buf[1000];
	int msg_size;
	while (1){
		if ((msg_size = read(conn_fd, buf, 1000)) > 0){
			buf[msg_size] = '\0';
			cout << buf;
			cout << "  msg_size: " << msg_size;
			sleep(1);
		} else if (msg_size == -1 && errno == 11){
			//perror("Reading error from USART: ");
			//cout << "errno: " << errno;
			//exit(1);
		} else {
			perror("Reading USART error:");
			exit(1);
		}

		sleep(1);
		transmit_motor_state(motors, 2, conn_fd);
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
	int ret_val = write(conn_fd, &NEW_MTR_STATE_OP, sizeof(NEW_MTR_STATE_OP));
	tcdrain(conn_fd);
	if (ret_val == -1){
		perror("Could not write motor op to connection: ");
	} else {
		cout << "Wrote: " << ret_val << " bytes." << endl;
	}

	char* pos = (char*) motors;
	char* term_pos = (char*) motors + (sizeof(motor_state) * num_motors);
	char empty_buf;
	tcflush(conn_fd, TCIOFLUSH);
	while(pos != term_pos){
		ret_val = write(conn_fd, pos, 1);
		if (ret_val == -1){
			perror("Could not write motor states to connection: ");
		} else {
			cout << "Wrote: " << ret_val << " bytes." << endl;
		}

		/*tcdrain(conn_fd);
		read(conn_fd, &empty_buf, 1);
		if (empty_buf - 48 != *pos){
			cout << "Error: sent: " << (int) *pos << " got: " << empty_buf << endl;
		} else {
			cout << empty_buf;
		}*/
		usleep(1000 * 50);
		pos++;
	}
	
}
