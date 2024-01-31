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
#include <stdbool.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define LEFT_EYE 0
#define RIGHT_EYE 1


void motor(uint8_t num, int8_t speed)
{ //num will be 0 or 1 corresponding to the left or right motor
  // speed will be a number from -100 (full speed reverse) to 
  //+100 (full speed forward).
        uint8_t adjusted_speed;
        if (num == 0) //wheel 0
        {
                adjusted_speed = (speed*3)/10 + 127;
                set_servo(LEFT_MOTOR, adjusted_speed); //fast
        }
        else    //wheel 1
        {
                adjusted_speed = (-speed*3)/10 + 127;
                set_servo(RIGHT_MOTOR, adjusted_speed); //fast
        }
}


void sensor_update()
{
        uint8_t left_eye = (analog(LEFT_EYE));
        lcd_cursor(0, 0);
        print_string("L: ");
        print_num(left_eye);
		print_string(" ");

        uint16_t right_eye = (analog(RIGHT_EYE));
        lcd_cursor(0, 1);
        print_string("R: ");
        print_num(right_eye);  
		print_string(" ");
}

#define BLACK_ANALOG 100
#define WHITE_ANALOG 150
#define K_P 10
#define K_D 10

void line_tracking()
{	
	uint8_t prev_left = 0;
	uint8_t prev_right = 0;
	uint8_t curr_left = 0;
	uint8_t curr_right = 0;

	uint8_t d_rate_of_change_left = 0;
	uint8_t d_rate_of_change_right = 0;

	uint8_t error_left = 0;
	uint8_t error_right = 0;

	uint8_t change_in_left = 0;
	uint8_t change_in_right = 0;


	while(1)
	{
        curr_left = (analog(LEFT_EYE));
        lcd_cursor(0, 0);
        print_string("L: ");
        print_num(curr_left);
		print_string(" ");

        curr_right = (analog(RIGHT_EYE));
        lcd_cursor(0, 1);
        print_string("R: ");
        print_num(curr_right);  
		print_string(" ");

		error_left = BLACK_ANALOG - error_left; //change if depending on black/white num
		error_right = BLACK_ANALOG - error_right;

		d_rate_of_change_left = curr_left - prev_left;
 		d_rate_of_change_right = curr_right - prev_right;

		//if right eye sees error, left wheel slows down
		//if left eye sees error, right wheel slows down

		change_in_left = (error_left * K_P) + (d_rate_of_change_left * K_D);
		change_in_right = (error_right * K_P) + (d_rate_of_change_right * K_D);


		//idk...
		motor(RIGHT_MOTOR, change_in_left);
		motor(LEFT_MOTOR, change_in_right);

		prev_left = curr_left;
		prev_right = curr_right;

	}



	
}



int main(void) {
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);

bool flag = false;

while(1) 
{  

	line_tracking();
}

return 0;
}
