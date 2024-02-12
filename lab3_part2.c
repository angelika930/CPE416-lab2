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
#define K_D 0.1
#define K_I 0.05
#define DEFAULT_SPEED 20
#define NUM_OF_ERROR_SAMPLES 5
#define NUM_OF_COLLECTED_SAMPLES 50
#define MOTOR_STABLE 127        //motors do not move at 127

struct motor_command {
        uint8_t left;
        uint8_t right;

} motor_command;


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

void add_to_error_array(int added_num, int error_samples[NUM_OF_ERROR_SAMPLES]) 
{
    for (int i = NUM_OF_ERROR_SAMPLES - 1; i > 0; i--) {
        error_samples[i] = error_samples[i - 1];
    }

    error_samples[0] = added_num;
}

float calculate_average_error(int error_samples[NUM_OF_ERROR_SAMPLES]) 
{
    int sum = 0;
    for (int i = 0; i < NUM_OF_ERROR_SAMPLES; i++) {
        sum += error_samples[i];
    }
   return (float)sum / NUM_OF_ERROR_SAMPLES;
}


struct motor_command compute_proportional(uint8_t curr_left, uint8_t curr_right)
{
        struct motor_command curr_motor_command =  {0};
        static int error_samples[NUM_OF_ERROR_SAMPLES] = {0};

        static int prev_error = 0;
        float derivative = 0;

        //find derivative
        int error = curr_left - curr_right;
        add_to_error_array(error, error_samples);
        derivative = calculate_average_error(error_samples);

        //PID equation
        int leftMotorSpeed = 45 + K_P * error + K_I * (error + prev_error) + K_D * derivative;	
        int rightMotorSpeed = 45 - K_P * error - K_I * (error + prev_error) - K_D * derivative;	

        //set the motors
        curr_motor_command.left = leftMotorSpeed;
        curr_motor_command.right = rightMotorSpeed;


        prev_error = error;

        return curr_motor_command;

}


void data_collection()
{/*
Measure the outside of the lines
If both sensors see the same value, assume they are on the correct side of the line
If a sensor does not see the corrrect value,  correct by the PDI 
*/
        int curr_left = 0;
        int curr_right = 0;

        struct motor_command sensor_val[50];
        int sample_count = 0;
        // sensor values 

        for (sample_count = 0; sample_count < NUM_OF_COLLECTED_SAMPLES; sample_count++)
        {
                curr_left = (analog(LEFT_EYE));
                curr_right = (analog(RIGHT_EYE));


                struct motor_command curr_reading = compute_proportional(curr_left, curr_right);
                sensor_val[sample_count] = curr_reading;
;

                clear_screen();
                lcd_cursor(0, 0);
                print_string("Data:");
                print_num(sample_count);  
                print_string("    ");

                lcd_cursor(0, 1);
                print_string("L");
                print_num(curr_left);  
                lcd_cursor(4, 1);
                print_string("R");
                print_num(curr_right);  


                if (button_delay_check(300))
                {//move to training
                        break;
                }
        }

        clear_screen();
        lcd_cursor(0, 0);
        print_string("victory!");

}


void line_seeking_PID()
{/*
Measure the outside of the lines
If both sensors see the same value, assume they are on the correct side of the line
If a sensor does not see the corrrect value,  correct by the PDI 
*/
    int curr_left = 0;
    int curr_right = 0;
    int prev_error = 0;
    int analog_samples[NUM_OF_ERROR_SAMPLES] = {0};
    int error = 0;
    float derivative = 0;
    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

        while (true) 
        {
                //pause motors if button was pressed

                if (button_delay_check(300))
                {//move to data collection
                        break;
                }

                // sensor values 
                curr_left = (analog(LEFT_EYE));
                curr_right = (analog(RIGHT_EYE));


                //find derivative
                error = curr_left - curr_right;
                add_to_error_array(error, analog_samples);
                derivative = calculate_average_error(analog_samples);

                //PID equation
		leftMotorSpeed = 30 + K_P * error + K_I * (error + prev_error) + K_P * derivative;	
		rightMotorSpeed = 30 - K_P * error - K_I * (error + prev_error) - K_P * derivative;	

                //set the motors
                motor(LEFT_MOTOR, leftMotorSpeed);
                motor(RIGHT_MOTOR, rightMotorSpeed);


                //print
                clear_screen();
                lcd_cursor(0, 0);
                print_string("Proportional:");

                lcd_cursor(0, 1);
                print_string("L");
                print_num(curr_left);
                lcd_cursor(4, 1);
                print_string("R");
                print_num(curr_right);  

                prev_error = error;
    }
}


void delay(u16 loop)
{
        u16 count = 0;
        while (count < loop)
        {
                _delay_ms(1);
                count ++;
        }
}



void display_tiltable_text()
{
    //start position
    print_num(416);

    int row = 0;
    int col = 4;

    while(1)
    {
        if ((get_accel_y() > 30 && get_accel_y()< 70 ) & (col > 0) )
        {//to the left
            col--;
        }
        if ( (get_accel_y() > 200 && get_accel_y()  < 250 ) & (col < 5) ) 
        {//to the right
            col++;
        }

        if ((get_accel_x() > 200 && get_accel_x()< 230 ))
        {// move down
            row = 1;
        }
        if ( (get_accel_x() > 20 && get_accel_x() < 70 ) ) 
        {//move up
            row = 0;
        }

        //print 416
        clear_screen();
        lcd_cursor(col, row);
        print_num(416);
        delay(50);

    }

}

void training_mode(int interations)
{//train neural network

}


int get_training_itertions()
{//use accel and a button press to return the desired number

}


int main(void) 
{
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);
   
   //porportional
   line_seeking_PID();


//data collection
   motor(0, 0);
   motor(1, 0);
   data_collection();

//get training interations


//training mode


//nerual network

return 0;
}

