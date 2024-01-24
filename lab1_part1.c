//Name: Christine Choi and Angelika Canete
//Lab 1 part 1
//Description: The onboard led 0 brightens, then dims gradually. The led 1 switches to do the same. The program repeats.

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

void delay(u16 loop)
{
        u16 count = 0;
        while (count < loop)
        {
                _delay_us(1);
                count ++;
        }
}



void dim(int led_num) 
{
        int brightness = 0; //brightness = 0, darkest at = 99
        led_on(led_num);
        
        while (brightness < 100)
        {
                int pause_count = 0; 
                while (pause_count < 100) 
                {       // Adjusts how long the brightness level stays
                        led_on(led_num);
                        delay(100-brightness);
                        led_off(led_num);
                        delay(brightness);
                        pause_count++;
                }
                brightness++;
        }


}

void brighten(int led_num) 
{
        int brightness = 100; //brightess = 0, darkest at = 99
        led_off(led_num);
        
        while (brightness > 0)
        {
                int pause_count = 0;
                while (pause_count < 100) 
                {       // Adjusts how long the brightness level stays
                        led_on(led_num);
                        delay(100-brightness);
                        led_off(led_num);
                        delay(brightness);
                        pause_count++;
                }
                brightness--;
        }
}


int main(void) {
   init();  //initialize board hardware


while(1) 
{
   brighten(0);
   dim(0);
   brighten(1);
   dim(1);
}

   return 0;
}
