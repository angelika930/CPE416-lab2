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


int main(void) {
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);


while(1) 
{

}

   return 0;
}
