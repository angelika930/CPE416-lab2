/*Name: Christine Choi and Angelika Canete
//Lab 1 part 4
//Description: 

The program that implements a line following robot. 
The robot will follow 3 tracks, a circle, square and an oval.
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
#define MOTOR_STABLE 127        //motors do not move at 127


struct motor_command {
        uint8_t left;
        uint8_t right;

} motor_command;

static struct motor_command sensor_val[50];
int a = 0;


void motor(uint8_t num, int8_t speed)
{ //num will be 0 or 1 corresponding to the left or right motor
  // speed will be a number from -100 (full speed reverse) to 
  //+100 (full speed forward)

        uint8_t adjusted_speed;
        if (num == 0) 
        {
                adjusted_speed = (speed*3)/10 + MOTOR_STABLE;
                set_servo(RIGHT_MOTOR, adjusted_speed); 
        }
        else    
        {
                adjusted_speed = (-speed*3)/10 + MOTOR_STABLE;
                set_servo(LEFT_MOTOR, adjusted_speed); 
        }
}

struct motor_command {
        uint8_t left;
        uint8_t right;

} motor_command;

void add_to_array(int analog_samples[], int added_num, int num_of_samples) 
{
    for (int i = num_of_samples - 1; i > 0; i--) {
        analog_samples[i] = analog_samples[i - 1];
    }

    analog_samples[0] = added_num;
}

float calculate_average() 
{
        struct motor_command curr_struct = sensor_val[a];
        int sum = 0;
        int  error = 0;
        int i = 0;
        if (a < NUM_OF_SAMPLES)
        {
                i = a;
        }
        else{
                i = 5;
        }    
        for (int b = i; b < NUM_OF_SAMPLES; b--) 
        {

                error = sensor_val[a-b].left - sensor_val[a-b].right;
                sum = error + sum;
        }
        
        return (float)sum / NUM_OF_SAMPLES;
}



void button_pause()
{//pauses the motor without needing a disconnect the wires
//reset to play again
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

struct motor_command compute_proportional(uint8_t curr_left, uint8_t curr_right)
{/*
Measure the outside of the lines
If both sensors see the same value, assume they are on the correct side of the line
If a sensor does not see the corrrect value,  correct by the PDI 
*/
    
        //pause motors if button was pressed
        button_pause();

        //find derivative
        uint8_t error = curr_left - curr_right;
        static uint8_t prev_error; 

        float derivative = calculate_average();

        //PID equation
        int leftMotorSpeed = 25 + K_P * error + K_I * (error + prev_error) + K_P * derivative;	
        int rightMotorSpeed = 25 - K_P * error - K_I * (error + prev_error) - K_P * derivative;	
        //PID equation
        int leftMotorSpeed = 25 + K_P * error + K_I * (error + prev_error) + K_P * derivative;	
        int rightMotorSpeed = 25 - K_P * error - K_I * (error + prev_error) - K_P * derivative;	

        //set the motors
        struct motor_command res;
        res.left = leftMotorSpeed;
        res.right = rightMotorSpeed;


        prev_error = error;
        return res;

    
}

///pretty sure either put a while loop in the function or in the main



int main(void) {
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);
int prev_error = 0;
uint8_t curr_left = 0;
uint8_t curr_right = 0;

while(a < 50) 
{  
        
                // sensor values 
        curr_left = (analog(LEFT_EYE));
        curr_right = (analog(RIGHT_EYE));

        struct motor_command curr_reading = compute_proportional(curr_left, curr_right);

        //Add output to an array of structs
        sensor_val[a] = curr_reading;
        _delay_ms(300);
        clear_screen();
        lcd_cursor(0, 0);

        print_num(sensor_val[a].left);
        lcd_cursor(0, 1);
        print_num(sensor_val[a].right);


        motor(0, sensor_val[a].left);
        motor(0, sensor_val[a].left);

        a++;

}
        clear_screen();
        lcd_cursor(0, 0);
        print_string("victory!");

return 0;
}
