#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "motor_state.h"
#include "usart.h"

//Connection Setup
#define BAUD_RATE 51
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

//Control Enums
#define RUN_NONE 0
#define RUN_ALL 1

const unsigned char all_ones = 255;
const unsigned char all_zeroes = 0;

void init();
void get_motor_cnt();
//void intake_data();
//void rx_motor_states();
struct motor_state* allocate_motors();
unsigned char get_motor_pin(struct motor_state motor);
void activate_motor(unsigned char motor_idx, struct motor_state* motors);
void deactivate_motor(unsigned char motor_idx, struct motor_state* motors);
void print_motor_state(struct motor_state* motors);

void kill_motors();
void zero_port_d();

unsigned char num_motors;
struct motor_state* motor_buf;
unsigned char* buf_pos;
unsigned char* buf_dest;
unsigned char filling_motor_buf;
unsigned char new_motor_state;
unsigned char op_code;
ISR(USART1_RX_vect)
{
	cli();
	//USART_send_string("Interrupt triggered!\r\n");
	if (filling_motor_buf){
		*buf_pos = UDR1;
		USART_transmit('f');
		if (buf_pos == buf_dest){
		       	filling_motor_buf = 0;
			new_motor_state = 1;
		} else {
			buf_pos++;
		}

	} else {
		op_code = UDR1;
		if (NEW_MTR_STATE_OP == op_code){
			filling_motor_buf = 1;
			buf_pos = (unsigned char*) motor_buf;
			USART_transmit('m');	
		} else { 
			USART_send_string("Error: Found unsupported OP! Terminating.\r\n");
			USART_transmit(op_code + 48);
			exit(1);
		}
	}

	sei();
}

unsigned char current_count = 0;
ISR(TIMER0_COMPA_vect)
{
	USART_transmit('t');
	if (current_count >= 14){
		current_count = 0;
		USART_transmit('r');
	}
	else {
		current_count++;
	}

	TCNT0 = 0;
}

int main(){
	init();
	MCUSR &= ~_BV(WDRF);
	wdt_disable();
	
	get_motor_cnt();
	struct motor_state* motors = allocate_motors();
	
/*	while(1){
		//PORTD = 0xFF;
		//PORTD = 0b01010101;
		//PORTD = 0b00001000;//Pins 3 and 2 show the same value! =(
		DDRD = 0xFF;
		PORTD = 0b10100000;
	}
	*/
	unsigned char i;
	while(1){
		current_count = current_count % 100;
		for (i = 0; i < num_motors; ++i){
			if (motor_is_enabled(motors + i) 
				&& get_motor_speed(motors + i) > current_count){
				USART_transmit(get_motor_speed(motors + i) + 65);
				activate_motor(i, motors);
			} else {
				deactivate_motor(i, motors);
			}
		}

		if (new_motor_state){
			memcpy(motors, motor_buf, sizeof(struct motor_state) * num_motors);
			new_motor_state = 0;
			USART_transmit('c');
			//print_motor_state(motors);
		}
	}
	
}

void get_motor_cnt()
{
	//TODO: implement a USART request for such a number
	//	send back an error if we do not get the OP_CODE
	//	to set this bit.
	num_motors = 2;
}

struct motor_state* allocate_motors()
{
	int motor_array_size = sizeof(struct motor_state) * num_motors;
	motor_buf = malloc(motor_array_size);
	struct motor_state* motors = malloc(motor_array_size);
	
	if (motors == NULL || motor_buf == NULL){
		USART_send_string("Error: Could not allocate all required motor storage information. Terminating.\r\n");
		exit(1);
	} else {
		buf_dest = (unsigned char*) motor_buf + motor_array_size - 1;
		memset(motors, 0, motor_array_size);
		memset(motor_buf, 0, motor_array_size);
		USART_send_string("Motor state structures allocated!\r\n");
	}

	return motors;
}

/*void rx_motor_states()
{
	USART_send_string("Receiving motor states.\r\n");
	unsigned char* pos = (unsigned char*) motors;
	unsigned char* term_pos = (unsigned char*) motors + (sizeof(struct motor_state) * num_motors);

	while (pos != term_pos){
		*pos = USART_receive();
		//USART_send_string("Got: ");
		USART_transmit(*pos + 48);
		//USART_transmit('\t');
		pos++;
	}

	USART_send_string("Got motor states!\r\n");
	//USART_transmit(40 + (term_pos - (unsigned char*) motors));
}
*/

unsigned char get_motor_pin(struct motor_state motor)
{
	if (motor.motor_state & LEFT_MOTOR){
		return 4;
	}
	
	return 6;	//It's the right motor.... No support for addtl motors.
}

void activate_motor(unsigned char motor_idx, struct motor_state* motors)
{
	unsigned char motor_pin = get_motor_pin(motors[motor_idx]);

	PORTD &= ~(1 << motor_pin);
	PORTD &= ~(1 << (motor_pin + 1));
	PORTD |= (motor_is_forward(motors + motor_idx) << (motor_pin + 1));

	USART_transmit('a');
}

void deactivate_motor(unsigned char motor_idx, struct motor_state* motors)
{
	unsigned char motor_pin = get_motor_pin(motors[motor_idx]);
	PORTD |= (1 << motor_pin);

	USART_transmit('d');
}

void init()
{
	GTCCR |= (1 << PSRSYNC);
	CPU_PRESCALE(0);

	USART_init(BAUD_RATE);
	USART_send_string("\n\nWe're online jack!\r\n");

	new_motor_state = 0;

	DDRD = 0xFF;
	PORTD = 0x50;

	sei();
	UCSR1B |= (1 << RXCIE1);
	if (UCSR1B & ~(1 << RXCIE1)){
		USART_send_string("Recieving Interrupt enabled.\r\n");
	}

	TCCR0B &= 0b11110000;
	TCCR0B |= 0b00000101;	//Max prescaling
	TCCR0A &= 0b11111100;	//Normal mode...
	OCR0A = 255;
	TIMSK0 |= 0b00000010;
	USART_send_string("Set timer to use CPU ticks, normal mode, with 255 comparison.");

}

void print_motor_state(struct motor_state* motors)
{
	int motor_array_size = sizeof(struct motor_state) * num_motors;
	unsigned char* pos = (unsigned char*) motors;
	int i;
	USART_send_string("State: ");
	for (i = 0; i < motor_array_size; ++i){
		USART_transmit(*(pos + i) + 48);
	}

	USART_send_string("\r\n");
}

void zero_port_d()
{
	DDRD = 0b11111111;
	PORTD = 0b00000000;
}

void kill_motors()
{
	PORTD = 0x00;
}
