/*Name: Christine Choi and Angelika Canete
//Lab 1 part 2
//Description: 


Write a program that:
- gradually spins the motors to full speed forward
- gradually slows the motors to a stop
- does the same in the reverse motor direction and continuously repeats
- while the program is running, print the motor speed on the screen

*/
#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>


void motor(uint8_t num, int8_t speed)
{
        if (speed == 30) {
                set_servo(num, 255);
        }
        else if (speed == -30) {
                set_servo(num, 97);    
        }
        else if (speed == 0) {
                set_servo(num, 127);
        }
        

}


int main(void) {
   init();  //initialize board hardware


while(1) 
{
        motor( 0, 0);
        motor(1, 0);
}

   return 0;
}
