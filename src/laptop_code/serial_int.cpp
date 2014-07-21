#include <cstdlib>
#include <iostream>
#include <string>
#include <SerialStream.h>

#include "../wonder_code/motor_state.h"

using std::cout;
using std::cin;
using std::endl;
using LibSerial::SerialStream;
using LibSerial::SerialStreamBuf;
using std::string;

void init_connection(SerialStream& stream);

int main(int argc, char** argv){
	ros::init(argc, argv, "motor_controller");
	ros::NodeHandle n;
	cout << "Initializing Serial Connection to Wunderboard!" << endl;
	SerialStream wunder_board;
	init_connection(wunder_board);

	string first_words;
	wunder_board >> first_words;
	//cout << "I heard: '" << first_words << "' from wunder_board" << endl;
/*	while(1){
		wunder_board >> first_words;
		cout << first_words << endl;
	}
*/

	//Send an arbitrary set of structs
	struct motor_state motors[2];
	motors[0].motor_name = LEFT_MOTOR;
	motors[0].enabled = MOTOR_EN;
	motors[0].dir = FORWARD;
	motors[0].power_percentage = 50;
	motors[1].motor_name = RIGHT_MOTOR;
	motors[1].enabled = MOTOR_EN;
	motors[1].dir = FORWARD;
	motors[1].power_percentage = 50;

	char op = NEW_MTR_STATE;
	wunder_board.write(&op, 1);
	wunder_board.write((char*) motors, sizeof(struct motor_state) * 2);

	return 0;
}

void init_connection(SerialStream& stream)
{

	stream.Open("/dev/ttyUSB0");
	if (stream.IsOpen()){
		cout << "\tStream open" << endl;
	} else {
		cout << "\tCould not open stream" << endl;
		exit(1);
	}

	//Connection Characteristics
	stream.SetBaudRate(SerialStreamBuf::BAUD_9600);
	stream.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	stream.SetNumOfStopBits(1);
	stream.SetParity(SerialStreamBuf::PARITY_NONE);
	stream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_HARD); //??
	stream.SetVMin(1000);

}

