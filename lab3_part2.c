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
#include <math.h>

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


/*


- how to calculate biases
- are we doing this right
- double check out output layer
- bias - 1 in hidden
- data collection
- what to pass into train neural network

*/


struct motor_command 
{
        uint8_t left;
        uint8_t right;
} motor_command;

struct pure_data 
{
    uint8_t left;
    uint8_t right;
} pure_data;

struct pure_data sensor_val[50];
int sample_count = 0;

struct neural_node {
    double w1;
    double w2;
    double w3;
    double bias;
} neural_node;

struct neural_node network[5];


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

uint8_t normalize(uint8_t value) 
{
    return (value * 100) / 255;
}

uint8_t denormalize(uint8_t value) 
{
        return (value * 255) / 100;
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
        curr_motor_command.left = normalize(leftMotorSpeed);
        curr_motor_command.right = normalize(rightMotorSpeed);


        prev_error = error;

        return curr_motor_command;

}

void network_init()
{
    //randomize weights on initialization
    network[0].w1 = rand()/ RAND_MAX;
    network[0].w2 = rand()/ RAND_MAX;
    network[0].bias = rand()/ RAND_MAX;

    network[1].w1 = rand()/ RAND_MAX;
    network[1].w2 = rand()/ RAND_MAX;
    network[1].bias = rand()/ RAND_MAX;

    network[2].w1 = rand()/ RAND_MAX;
    network[2].w2 = rand()/ RAND_MAX;
    network[2].bias = rand()/ RAND_MAX;


    //output layer weights
    network[3].w1 = rand()/ RAND_MAX;
    network[3].w2 = rand()/ RAND_MAX;
    network[3].w3 = rand()/ RAND_MAX;
    network[3].bias = rand()/ RAND_MAX;

    network[4].w1 = rand()/ RAND_MAX;
    network[4].w2 = rand()/ RAND_MAX;
    network[4].w3 = rand()/ RAND_MAX;
    network[4].bias = rand()/ RAND_MAX;

}

void data_collection()
{/*
Measure the outside of the lines
If both sensors see the same value, assume they are on the correct side of the line
If a sensor does not see the corrrect value,  correct by the PDI 
*/
        int curr_left = 0;
        int curr_right = 0;
        // sensor values 

        for (sample_count = 0; sample_count < NUM_OF_COLLECTED_SAMPLES; sample_count++)
        {
                curr_left = (analog(LEFT_EYE));
                curr_right = (analog(RIGHT_EYE));
            
                struct pure_data curr_reading;
                curr_reading.left = curr_left;
                curr_reading.right = curr_right;

                sensor_val[sample_count] = curr_reading;

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

void train_neural_network(uint8_t curr_left, uint8_t curr_right) 
{
    //represents partial derivate of E total
    //float Etotal = 2 * 0.5 * (out - target)-1;
    
    //partial derivative of weight
    //float der_weight    how do i find a partial derivative of a number???

    //new weight = old weight - LEARN_RATE * (Etotal / der_weight)
}

void line_seeking_PID()
{/*
Measure the outside of the lines
If both sensors see the same value, assume they are on the correct side of the line
If a sensor does not see the corrrect value, correct by the PDI 
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

void training_mode(int interations)
{//train neural network

}

int get_training_itertions()
{
    //recieve user input by tiling 
    lcd_cursor(0, 0);
    print_string("<=-  +=>");
    int iterations = 0;

    while(1)
    {
        if ((get_accel_y() > 30 && get_accel_y()< 70 ))
        {//to the left
            iterations--;
        }
        if ( (get_accel_y() > 200 && get_accel_y()  < 250 )) 
        {//to the right
            iterations++;
        }
        if ((get_accel_x() > 200 && get_accel_x()< 230 ))
        {// move down
            iterations = iterations - 10;
        }
        if ( (get_accel_x() > 20 && get_accel_x() < 70 ) ) 
        {//move up
            iterations = iterations + 10;
        }

        if (iterations < 0)
        {//no negatives!
                iterations = 0;
        }

        //print 416
        lcd_cursor(0, 1);
        print_num(iterations);
        print_string("       ");
        if (button_delay_check(50))
        {
                return iterations;
        }

    }
    return 0;

}

    double h1_out ;
    double h2_out ;
    double h3_out ;
    double o1_out ;
    double o2_out ;

void update_weights(struct motor_command target, struct motor_command out, int sensor_left, int sensor_right)
{
    int target_value = target.right + target.left;
    int out_value = out.right + out.left;
    

    //----hidden----------------------
    int new0_w1 = network[0].w1 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* sensor_left);
    int new0_w2 = network[0].w2 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* sensor_right);

    int new0_bias; 

    int new1_w1 = network[1].w1 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* sensor_left);
    int new1_w2 = network[1].w2 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* sensor_right);
    int new1_bias; 

    int new2_w1 = network[2].w1 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* sensor_left);
    int new2_w2 = network[2].w2 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* sensor_right);
    int new2_bias; 

    //----output----------------------
    int new3_w1 =  network[3].w1 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* (h1_out - 1));
    int new3_w2  =  network[3].w2 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* (h2_out) - 1);
    int new3_w3 =  network[3].w3 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* (h3_out) - 1);
    int new3_bias = -1;

    int new4_w1 =  network[4].w1 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* (h1_out - 1));
    int new4_w2 = network[4].w2 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* (h2_out- 1));
    int new4_w3 =  network[4].w3 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* (h3_out- 1));
    int new4_bias = -1;
}
    //sigmoid calculation


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

int main(void) 
{
   init();  //initialize board hardware
   motor(0, 0);
   motor(1, 0);
   
   //proportional
   line_seeking_PID();

    //data collection
    motor(0, 0);
    motor(1, 0);
    data_collection();
    network_init();

    //get training iterations
    int iterations = get_training_itertions();
    network_init();

    for (int i = 0; i < iterations; i++) 
    {
        for (int j = 0; j < NUM_OF_COLLECTED_SAMPLES; j++) 
        {
        struct motor_command target = compute_proportional(sensor_val[j].left, sensor_val[j].right);
        struct motor_command out = compute_neural_network(sensor_val[j].left, sensor_val[j].right);

        update_weights(target, out, sensor_val[j].left, sensor_val[j].right);


        }
        
    }

    //training mode

    training_mode(iterations);



//neural network

return 0;
}

