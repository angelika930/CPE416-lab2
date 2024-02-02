
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
#include <stdlib.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define LEFT_EYE 0
#define RIGHT_EYE 1

#define IS_BETWEEN(x, a, b) ((x) >= (a) && (x) <= (b))
#define K_P 0.2
#define K_D 0
#define K_I 0.05
#define DEFAULT_SPEED 20
#define NUM_OF_SAMPLES 5

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


void add_to_array(int analog_samples[], int added_num, int num_of_samples) 
{
   //shift
    for (int i = num_of_samples - 1; i > 0; i--) {
        analog_samples[i] = analog_samples[i - 1];
    }

    analog_samples[0] = added_num;
}

float calculate_average(int analog_samples[NUM_OF_SAMPLES]) 
{
    int sum = 0;
    for (int i = 0; i < NUM_OF_SAMPLES; i++) {
        sum += analog_samples[i];
    }
   return (float)sum / NUM_OF_SAMPLES;
}

void button_pause()
{
        if (get_btn())
        {
                motor(0, 0);
                motor(1, 0);
                clear_screen();
                print_string("paused");
                while(1)
                        {
                                
                        }
        }
}

void line_seeking()
{
    int curr_left = 0;
    int curr_right = 0;
    int prev_error = 0;
    int analog_samples[NUM_OF_SAMPLES] = {0};

        while (true) 
        {
                button_pause();

                // print 
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

                int error = curr_left - curr_right;
                add_to_array(analog_samples, error, NUM_OF_SAMPLES);
                float derivative = calculate_average(analog_samples);

//                int leftMotorSpeed = 25 + K_P * error + K_D * derivative + K_I * (error + prev_error);
  //              int rightMotorSpeed = 25 - K_P * error - K_D * derivative - K_I * (error + prev_error);
	
		int leftMotorSpeed = 25 + K_P * error + K_I * (error + prev_error) + K_P * derivative;	
		int rightMotorSpeed = 25 - K_P * error - K_I * (error + prev_error) - K_P * derivative;	

//		if (error > 5 ) {

  //      		motor(RIGHT_MOTOR, -leftMotorSpeed);
    //    		motor(LEFT_MOTOR, rightMotorSpeed);
//		}
//		else {

        		// Set motor speeds
        		motor(LEFT_MOTOR, leftMotorSpeed);
        		motor(RIGHT_MOTOR, rightMotorSpeed);
//		}

        prev_error = error;

    }
}

int main(void) {
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);

while(1) 
{  
        line_seeking();
        
}

return 0;
}
