#include <avr/io.h>

int main(int arc, char** argv){
	DDRB = 1 << DDB1;
	PORTB = 1 << PB1;

	return 0;
}
