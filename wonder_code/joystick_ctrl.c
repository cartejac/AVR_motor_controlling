#include <stdlib.h>
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
void rx_motor_states();
void allocate_motors();
void activate_motor(unsigned char motor_idx);
void deactivate_motor(unsigned char motor_idx);

void kill_motors();
void zero_port_d();

unsigned char op_code;
ISR(USART1_RX_vect)
{
	USART_send_string("Interrupt triggered!\r\n");
}
void intake_data()
{
	USART_send_string("Receiving Interrupt triggered.\r\n");
	op_code = USART_receive();
	USART_send_string("Received: ");
	USART_transmit(op_code + 48);
	USART_send_string("\r\n");
	if (NEW_MTR_STATE_OP == op_code){
		rx_motor_states();
	} else { 
		USART_send_string("Error: Found unsupported OP! Terminating.\r\n");
		exit(1);
	}
}

//TODO:Implement the clock interrupt to enable
//	variable power functionality.
unsigned char current_count = 0;


unsigned char num_motors;
struct motor_state* motors;

int main(){
	init();
	MCUSR &= ~_BV(WDRF);
	wdt_disable();
	
	get_motor_cnt();
	allocate_motors();
	
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
			if (motors[i].enabled == MOTOR_EN 
				&& motors[i].power_percentage > current_count){
				activate_motor(i);
			} else {
				deactivate_motor(i);
			}
		}

		if (USART_available()){
			USART_send_string("Data available\r\n");
			intake_data();
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

void allocate_motors()
{
	motors = malloc(sizeof(struct motor_state) * num_motors);
	if (motors == NULL){
		USART_send_string("Error: Could not allocate all required motors. Terminating.\r\n");
		exit(1);
	} else {
		USART_send_string("Motor state structures allocated!\r\n");
	}

}

void rx_motor_states()
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

void activate_motor(unsigned char motor_idx)
{
	PORTD |= (0 << motors[motor_idx].motor_name);
	PORTD &= ~(1 << (motors[motor_idx].motor_name + 1));
	PORTD |= (motors[motor_idx].dir << (motors[motor_idx].motor_name + 1));
}

void deactivate_motor(unsigned char motor_idx)
{
	PORTD &= ~(1 << motors[motor_idx].motor_name);
}

void init()
{
	CPU_PRESCALE(0);

	USART_init(BAUD_RATE);
	USART_send_string("\n\nWe're online jack!\r\n");

	sei();
	UCSR1B |= (1 << RXCIE1);
	if (UCSR1B & ~(1 << RXCIE1)){
		USART_send_string("Recieving Interrupt enabled.");
	}

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
