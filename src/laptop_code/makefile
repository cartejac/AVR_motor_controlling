CC=g++
FLAGS=-g -Wall

default: termios_interface

libserial_interface: serial_int.cpp ../wonder_code/motor_state.h
	${CC} ${FLAGS} serial_int.cpp -lserial -o libserial_interface

termios_interface: termios_int.cpp ../wonder_code/motor_state.h
	${CC} ${FLAGS} termios_int.cpp -o termios_interface
