// #include "globals.h"
// #include <util/delay.h>
// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include <stdio.h>
// #include <stdbool.h>
// #include <stdlib.h>
// #include <math.h>

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
#define LEARN_RATE .145

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

void data_collection()
{/*
Measure the outside of the lines
If both sensors see the same value, assume they are on the correct side of the line
If a sensor does not see the corrrect value,  correct by the PDI 
*/
        int curr_left = 0;
        int curr_right = 0;
        int fake_right = 50;
        // sensor values 

        for (int sample_count = 0; sample_count < NUM_OF_COLLECTED_SAMPLES; sample_count++)
        {
                curr_left = sample_count;
                curr_right = fake_right;
            
                struct pure_data curr_reading;
                curr_reading.left = curr_left;
                curr_reading.right = curr_right;

                sensor_val[sample_count] = curr_reading;

             //   clear_screen();
             //   lcd_cursor(0, 0);
                print("Data:");
                print(sample_count);  
                print("    ");

            //    lcd_cursor(0, 1);
                print("L");
                print(curr_left);  
             //   lcd_cursor(4, 1);
                print("R");
                print(curr_right);  

                if (button_delay_check(300))
                {//move to training
                        break;
                }
                fake_right--;
        }
     //   clear_screen();
      //  lcd_cursor(0, 0);
        print("victory!");

}
struct motor_command compute_neural_network(uint8_t curr_left, uint8_t curr_right) 
{
    //calculate net value
    double h1_net = (curr_left * network[0].w1 + curr_right * network[0].w2) - network[0].bias;
    double h2_net = (curr_left * network[1].w1 + curr_right * network[1].w2) - network[1].bias;
    double h3_net = (curr_left * network[2].w1 + curr_right * network[2].w2) - network[2].bias;

    //sigmoid calculation
     h1_out = 1/(1 + exp(-(h1_net)));
     h2_out = 1/(1 + exp(-(h2_net)));
     h3_out = 1/(1 + exp(-(h3_net)));

    //outer layer equation
    double o1_net = (h1_out * network[3].w1 + h2_out * network[3].w2 + h3_out * network[3].w3) - network[3].bias;
    double o2_net = (h1_out * network[4].w1 + h2_out * network[4].w2 + h3_out * network[4].w3) - network[4].bias;

    //sigmoid calculation
     o1_out = 1/(1 + exp(-(o1_net)));
     o2_out = 1/(1 + exp(-(o2_net)));

    //outputs predicted motor values
    struct motor_command computed_nodes;
    computed_nodes.left = o1_out;
    computed_nodes.right = o2_out;

    return computed_nodes;
}
uint8_t denormalize(uint8_t value) 
{
        return (value * 255) / 100;
}


int main(void) {
    
    for(int a = 0; a < sensor_val.size(); a++) {
       print(denormalize(a);

    }
}
