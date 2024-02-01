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
                set_servo(RIGHT_MOTOR, adjusted_speed); //fast
        }
        else    //wheel 1
        {
                adjusted_speed = (-speed*3)/10 + 127;
                set_servo(LEFT_MOTOR, adjusted_speed); //fast
        }
}

#define UPPER_BOUND 100
#define LOWER_BOUND 50
#define K_P 10
#define K_D 10
#define DEFAULT_SPEED 10
#define NUM_OF_SAMPLES 10

//array handling
int * add_to_array(int analog_samples[NUM_OF_SAMPLES], int added_num)
{//add added_num to the front of the array, remove the oldest num

int i = 0;
for (i = 0; i < NUM_OF_SAMPLES; i++)
{
        analog_samples[i] = analog_samples[i-1];
}
analog_samples[0] = added_num;

return analog_samples;
}


void line_tracking()
{	
        /*
        int analog_samples[NUM_OF_SAMPLES] = 0;

	uint8_t prev_left = 0;
	uint8_t prev_right = 0;



	uint8_t d_rate_of_change_left = 0;
	uint8_t d_rate_of_change_right = 0;
        */
       	uint8_t curr_left = 0;
	uint8_t curr_right = 0;

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
                print_string("    ");

                curr_right = (analog(RIGHT_EYE));
                lcd_cursor(0, 1);
                print_string("R: ");
                print_num(curr_right);  

		print_string("    ");

                if (curr_left > UPPER_BOUND)
                {
                        error_left = UPPER_BOUND - curr_left;
                }
                else if (curr_left < LOWER_BOUND)
                {
                        error_left = LOWER_BOUND - curr_left;
                }

                error_left = error_left * 0.3;

                motor(RIGHT_MOTOR, DEFAULT_SPEED - error_left);

/////////////////////////////////////////



                if (curr_right > UPPER_BOUND)
                {
                        error_right = UPPER_BOUND - curr_right;
                }
                else if (curr_right < LOWER_BOUND)
                {
                        error_right = LOWER_BOUND - curr_right;
                }

                error_right = error_right * 0.2;

                motor(LEFT_MOTOR, DEFAULT_SPEED - error_right);

	}



	
}

void darkness_seeking()
{
        uint8_t curr_left = 0;
	uint8_t curr_right = 0;

        curr_left = (analog(LEFT_EYE));
        lcd_cursor(0, 0);
        print_string("L: ");
        print_num(curr_left);
        print_string("    ");

        curr_right = (analog(RIGHT_EYE));
        lcd_cursor(0, 1);
        print_string("R: ");
        print_num(curr_right);  

        print_string("    ");

        if (curr_right > curr_left)
        {
                motor(LEFT_MOTOR, 20);
                motor(RIGHT_MOTOR, 10);
        }
        else
        {
                motor(LEFT_MOTOR, 10);
                motor(RIGHT_MOTOR, 20);
        }


}


int main(void) {
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);

bool flag = false;

while(1) 
{  
        

	darkness_seeking();
}

return 0;
}
