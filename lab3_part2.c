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
#define LEARN_RATE .10


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
        double left;
        double right;
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

struct neural_node network[5] = {0};

    double h1_out = 0;
    double h2_out = 0;
    double h3_out = 0;
    double o1_out = 0;
    double o2_out = 0;

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

double normalize(uint8_t value) 
{
    return (value / 100);
}

uint8_t denormalize(double value) 
{
        return (value * 100);
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
    network[0].w1 =(double) rand()/ RAND_MAX;
    network[0].w2 = (double)rand()/ RAND_MAX;
    network[0].bias = (double)rand()/ RAND_MAX;

    network[1].w1 =(double) rand()/ RAND_MAX;
    network[1].w2 = (double)rand()/ RAND_MAX;
    network[1].bias = (double)rand()/ RAND_MAX;

    network[2].w1 = (double)rand()/ RAND_MAX;
    network[2].w2 = (double)rand()/ RAND_MAX;
    network[2].bias = (double)rand()/ RAND_MAX;


    //output layer weights
    network[3].w1 = (double)rand()/ RAND_MAX;
    network[3].w2 = (double)rand()/ RAND_MAX;
    network[3].w3 = (double)rand()/ RAND_MAX;
    network[3].bias = (double)rand()/ RAND_MAX;

    network[4].w1 = (double)rand()/ RAND_MAX;
    network[4].w2 = (double)rand()/ RAND_MAX;
    network[4].w3 = (double)rand()/ RAND_MAX;
    network[4].bias = (double)rand()/ RAND_MAX;


}

int data_collection()
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
                        return sample_count;
                }
        }
        clear_screen();
        lcd_cursor(0, 0);
        print_string("victory!");
    return sample_count;

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
                if (get_btn())
                {
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
		leftMotorSpeed = 45 + K_P * error + K_I * (error + prev_error) + K_D * derivative;	
		rightMotorSpeed = 45 - K_P * error - K_I * (error + prev_error) - K_D * derivative;	

                //set the motors
                motor(LEFT_MOTOR, leftMotorSpeed);
                motor(RIGHT_MOTOR, rightMotorSpeed);

                //print
                clear_screen();
                lcd_cursor(0, 0);
                print_string("L: ");
                print_num(leftMotorSpeed);
                print_string("    ");
                lcd_cursor(0, 1);
                print_string("R: ");
                print_num(rightMotorSpeed);  
                print_string("    ");

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


int get_training_itertions()
{
    //receive user input by tiling 
    clear_screen();
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


void train_neural_network(struct motor_command target, struct motor_command out, int sensor_left, int sensor_right)
{
    //recieves datapoints captured outputs update neuralnetwork

    //----hidden----------------------
    double new0_w1 = network[0].w1 - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w1)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w1))
    * (h1_out * (1-h1_out)) * sensor_left);
    double new0_w2 = network[0].w2 - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w1)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w1))
    * (h1_out * (1-h1_out)) * sensor_right);
    double new0_bias = network[0].bias - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w1)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w1))
    * (h1_out * (1-h1_out)) * -1);

    double new1_w1 = network[1].w1 - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w2)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w2))
    * (h2_out * (1-h2_out)) * sensor_left);
    double new1_w2 = network[1].w2 - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w2)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w2))
    * (h2_out * (1-h2_out)) * sensor_right);
    double new1_bias = network[1].bias - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w2)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w2))
    * (h2_out * (1-h2_out)) * -1);


    double new2_w1 = network[2].w1 - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w3)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w3))
    * (h3_out * (1-h3_out)) * sensor_left);
    double new2_w2 = network[2].w2 - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w3)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w3))
    * (h3_out * (1-h3_out)) * sensor_right);
    double new2_bias = network[2].bias - LEARN_RATE * 
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w3)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w3))
    * (h3_out * (1-h3_out)) * -1);

    //----output----------------------
    double new3_w1 =  network[3].w1 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* (h1_out));
    double new3_w2  =  network[3].w2 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* (h2_out));
    double new3_w3 =  network[3].w3 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* (h3_out));
    double new3_bias = network[3].w3 - LEARN_RATE * 
    ((out.left - target.left)* (out.left * (1 - out.left))* (-1));

    double new4_w1 =  network[4].w1 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* (h1_out));
    double new4_w2 = network[4].w2 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* (h2_out));
    double new4_w3 =  network[4].w3 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* (h3_out));
    double new4_bias = network[4].w3 - LEARN_RATE * 
    ((out.right - target.right)* (out.right * (1 - out.right))* (-1));


    //update
    network[0].w1 = new0_w1;
    network[0].w2 = new0_w2;
    network[0].bias = new0_bias;

    network[1].w1 = new1_w1;
    network[1].w2 = new1_w2;
    network[1].bias = new1_bias;

    network[2].w1 = new2_w1;
    network[2].w2 = new2_w2;
    network[2].bias = new2_bias;

    network[3].w1 = new3_w1;
    network[3].w2 = new3_w2;
    network[3].w2 = new3_w3;
    network[3].bias = new3_bias;

    network[4].w1 = new4_w1;
    network[4].w2 = new4_w2;    
    network[4].w2 = new4_w3;
    network[4].bias = new4_bias;

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
    computed_nodes.left = (o1_out);
    computed_nodes.right = (o2_out);

    return computed_nodes;
}

void run_motors(struct motor_command out) 
{
    lcd_cursor(0, 0);
    print_string("L is ");
    print_num(denormalize(out.left));
    print_string("     ");
    lcd_cursor(0, 1);
    print_string("R is ");
    print_num(denormalize(out.right));
    print_string("     ");

    motor(0, denormalize(out.left));
    motor(1, denormalize(out.right));
}


enum state
{
INIT_STATE, PID_STATE, DATA_STATE, INPUT_STATE, TRAIN_STATE, FORWARD_STATE
};

int main(void) 
{
    enum state current_state = INIT_STATE;
    int epochs = 0;
    int data_collected = 0;

   while(1)
   {
    switch(current_state)
    {
        case INIT_STATE:
        {
            init();  //initialize board hardware
            network_init();
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            current_state = PID_STATE;
        }
        break;

        case PID_STATE:
        {
            line_seeking_PID();
            _delay_ms(150);  //delay for debouncing
            current_state = DATA_STATE;
        }
        break;

        case DATA_STATE:
        {   //collecting data
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            data_collected = data_collection();
            _delay_ms(70);  //delay for debouncing

            current_state = INPUT_STATE;
        }
        break;

        case INPUT_STATE:
        {
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            epochs = get_training_itertions();
            current_state = TRAIN_STATE;
            _delay_ms(70);  //delay for debouncing
        }
        break;

        case TRAIN_STATE:
        {
            clear_screen();
            lcd_cursor(0,0);
            print_string("Training: ");
            for (int i = 0; i < epochs; i++) 
            {
                for (int j = 0; j < 1; j++) 
                {
                    struct motor_command target = compute_proportional(sensor_val[j].left, sensor_val[j].right);
                    struct motor_command out = compute_neural_network(sensor_val[j].left, sensor_val[j].right);
                    train_neural_network(target, out, sensor_val[j].left, sensor_val[j].right);

                }
                delay(10);
                lcd_cursor(0,1);
                print_num(i);
                print_string("   ");
            }
            current_state = FORWARD_STATE;
        }

        break;

        case FORWARD_STATE:
        {

            print_string("hello");
            struct motor_command current = compute_neural_network(analog(LEFT_EYE), analog(RIGHT_EYE));
            run_motors(current);
            
            if (button_delay_check(30))
            {
                delay(50);
                current_state = INPUT_STATE;
            } 
            else
            {
                current_state = FORWARD_STATE;

            }  
        }
        break;
    }
   }

return 0;
}

