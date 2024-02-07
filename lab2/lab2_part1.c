/*Name: Christine Choi and Angelika Canete
//Lab 2 part 1
//Description: 


This program gradually spins the motors to full speed forward, 
gradually slows the motors to a stop, does the same in the reverse motor direction,
and continuously repeats. Additionally, while the program is running, 
it prints the motor speed on the screen.
*/

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define LEFT_EYE 0
#define RIGHT_EYE 1

void motor(uint8_t num, int8_t speed)
{ //num will be 0 or 1 corresponding to the left or right motor
  // speed will be a number from -100 (full speed reverse) to 
  //+100 (full speed forward).
        uint8_t adjusted_speed;
        if (num == LEFT_MOTOR) //wheel 0
        {
                adjusted_speed = (speed*3)/10 + 127;
                lcd_cursor(0,0);
                print_string("0: ");
                print_num(adjusted_speed);
                set_servo(0, adjusted_speed); //fast
        }
        else    //wheel 1
        {
                adjusted_speed = (-speed*3)/10 + 127;
                lcd_cursor(0, 1);
                print_string("1: ");
                print_num(adjusted_speed);
                set_servo(1, adjusted_speed); //fast
        }
}



void full_speed(uint8_t direction)
{//gradually spins the motors to full speed forward (1)
//or backwards (0)
int8_t max = (direction)?(100):(-100);

int8_t i = 0;
while (i != max)
{ 
        motor(LEFT_MOTOR, i);
        motor(RIGHT_MOTOR, i);
        _delay_ms(100);
        if (direction)
                i++;
        else
                i--;
}
}

void gradually_stop(uint8_t direction)
{//gradually slows the motors to a stop
//forward (1) or backwards (0)
int8_t i = (direction)?(100):(-100);
while (i != 0)
{ 
        motor(LEFT_MOTOR, i);
        motor(RIGHT_MOTOR, i);
        _delay_ms(100);
        if (direction)
                i--;
        else
                i++;
}
}

int main(void) {
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);


while(1) 
{
        full_speed(0);
        gradually_stop(0);

        full_speed(1);
        gradually_stop(1);
}

   return 0;
}
