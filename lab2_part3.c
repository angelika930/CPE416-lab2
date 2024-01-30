
#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>


u16 button_delay_check(u16 loop)
{
    //acts as a delay_ms function that also checks for a button press

        u16 count = 0;
        int button_flag = 0;
        while (count < loop)
        {
            if (get_btn()==1)
            {
                button_flag = 1;
            }
            _delay_ms(1);
            count ++;
        }
        return button_flag;
}

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


void attraction(int state1, int state2) {
	if (analog(3) > state1) {
		motor(1, (state1 - (state1 * .25)));
		motor(0, (state1 - (state1 *.70)));
		state1 = analog(3);
	//	_delay_ms(500);
	}
//	else if (analog(3) < state1) {
//		motor(1, (state1 - (state1 * .70)));
//		motor(0, (state1 - (state1 * .25)));
//		state1 = analog(3);
//	}
	else {
		state1 = 70;
		motor(1, state1);
	}
	if (analog(4) > state2) {
		motor(0, (state2 - (state2 *.25)));
		motor(1, (state2 - (state2 *.70)));
		state2 = analog(4);
	//	_delay_ms(500);
	}
//	else if (analog(4) <= state2) {

//		motor(0, (state2 - (state2*.70)));
//		motor(1, (state2 - (state2*.25)));
//		state2 = analog(4);
//	}
	else {
		state2 = 70;
		motor(0, state2);
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


int main(void) {
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);

int fstate1 = 50;
int fstate2 = 50;

int astate1 = 50;
int astate2 = 50;

bool flag = false;

while(1) 
{  
	/*
	while (flag == false) {
		_delay_ms(300);
		clear_screen();
		print_string("Fear");
		fear(fstate1, fstate2);

		if (button_delay_check(1) == 1) {
			flag = true;
			break;
		}
	}
	while (flag) {
		_delay_ms(300);
		clear_screen();
		print_string("Aggression");
		aggression(astate1, astate2);

		if (button_delay_check(1) == 1) {
			flag = false;
			break;
		}
	}

//	motor(0,0);
//	motor(1,0);

        */
	attraction(astate1, astate2);


}

return 0;
}
