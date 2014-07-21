TARGET=joystick_ctrl.c
CC=avr-gcc
MODEL=at90usb1287
FLAGS=-g -Os -Wall
OBJS=usart.elf main_target.elf

default:program clean

program: ${OBJS}
	${CC} ${FLAGS} -mmcu=${MODEL} ${OBJS} -o ${TARGET}.elf 
	avr-objcopy -j .text -j .data -O ihex ${TARGET}.elf ${TARGET}.hex
	sudo dfu-programmer at90usb1287	erase 
	sudo dfu-programmer at90usb1287 flash ${TARGET}.hex
	#sudo dfu-programmer at90usb1287 reset ##Doesn't quite work...

main_target.elf: ${TARGET} motor_state.h
	${CC} ${FLAGS} -c -mmcu=${MODEL} ${TARGET} -o main_target.elf 

usart.elf: usart.h usart.c
	${CC} ${FLAGS} -c -mmcu=${MODEL} usart.c -o usart.elf

clean:
	rm ${OBJS} ${TARGET}.hex

test:
	${TARGET}=port_test.c
	program
	clean
