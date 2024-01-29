/*Name: Christine Choi and Angelika Canete
//Lab 1 part 2
//Description: 


Write a program that:
- gradually spins the motors to full speed forward
- gradually slows the motors to a stop
- does the same in the reverse motor direction and continuously repeats
- while the program is running, print the motor speed on the screen
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
        motor(0, i);
        motor(1, i);
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
        motor(0, i);
        motor(1, i);
        _delay_ms(100);
        if (direction)
                i--;
        else
                i++;
}
}

int main(void) {
   init();  //initialize board hardware
   motor(0, 100);
   motor(1, 100);


while(1) 
{
        full_speed(1);
        gradually_stop(1);

        full_speed(0);
        gradually_stop(0);


}

   return 0;
}
