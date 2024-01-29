/*Name: Christine Choi and Angelika Canete
//Lab 1 part 2
//Description: 

The program that implements Braitenberg vehicles 2a and 2b. 
Pressing the on-board button should toggle between the 2 vehicles
The display  show whichs vehicle is running

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
*/

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>


void motor(uint8_t num, int8_t speed)
{ //num will be 0 or 1 corresponding to the left or right motor
  // speed will be a number from -100 (full speed reverse) to 
  //+100 (full speed forward).
        uint8_t adjusted_speed;
        if (num == 0) //wheel 0
        {
                adjusted_speed = (speed*3)/10 + 127;
                set_servo(0, adjusted_speed); //fast
        }
        else    //wheel 1
        {
                adjusted_speed = (-speed*3)/10 + 127;
                set_servo(1, adjusted_speed); //fast
        }
}


void sensor_update()
{
        uint8_t left_eye = (analog(3));
        lcd_cursor(0, 0);
        print_string("L: ");
        print_num((uint16_t)left_eye);
	print_string(" ");

        uint16_t right_eye = (analog(4));
        lcd_cursor(0, 1);
        print_string("R: ");
        print_num(right_eye);  
	print_string(" ");
}
void fear(int state1, int state2) {
	if (analog(3) > state1) {
		motor(1, (state1 + state1 * .10));
		state1 = analog(3);
	//	_delay_ms(500);
	}
	else {
		motor(1, (state1 - state1 * .10));
		state1 = analog(3);
	}
	if (analog (4) > state2) {
		motor(0, (state2 + state2*.10));
		state2 = analog(4);
	//	_delay_ms(500);
	}
	else {

		motor(0, (state2 - state2*.10));
		state2 = analog(4);
	}

}

int main(void) {
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);

int state1 = 50;
int state2 = 50;
while(1) 
{  
        sensor_update();
	fear(state1, state2);

        


}

return 0;
}
