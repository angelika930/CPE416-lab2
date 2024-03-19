"""CPE416 Sample Controller"""

from controller import Robot, Motor
import random
import math

from pynput import keyboard
from pynput.keyboard import Key
import keyboard  # using module keyboard

#################################################################

kp = 0.03
LEARN_RATE = 0.37

hidden_1 = {'w1': 0, 'w2': 0, 'bias': 0}
hidden_2 = {'w1': 0, 'w2': 0, 'bias': 0}
hidden_3 = {'w1': 0, 'w2': 0, 'bias': 0}

output_1 = {'w1': 0, 'w2': 0, 'w3': 0, 'bias': 0}
output_2 = {'w1': 0, 'w2': 0, 'w3': 0, 'bias': 0}


#globals
h1_out = 0
h2_out = 0
h3_out = 0

def motor(wheel, speed):
    #-100 -> -6.38
    #0 -> 0
    #100 -> 6.28
    if (wheel == 0): #left
        adjusted_speed = (speed * 6.28 /100)
        left_motor.setVelocity(adjusted_speed)    
    elif (wheel == 1): #right
        adjusted_speed = (speed * 6.28 /100)
        right_motor.setVelocity(adjusted_speed)    



def network_init():
    hidden_1["w1"] = random.uniform(0, 1)
    hidden_1["w2"] = random.uniform(0, 1)
    hidden_1["bias"] = random.uniform(0, 1)

    hidden_2["w1"] = random.uniform(0, 1)
    hidden_2["w2"] = random.uniform(0, 1)
    hidden_2["bias"] = random.uniform(0, 1)

    hidden_3["w1"] = random.uniform(0, 1)
    hidden_3["w2"] = random.uniform(0, 1)
    hidden_3["bias"] = random.uniform(0, 1)

    output_1["w1"] = random.uniform(0, 1)
    output_1["w2"] = random.uniform(0, 1)
    output_1["w3"] = random.uniform(0, 1)
    output_1["bias"] = random.uniform(0, 1)

    output_2["w1"] = random.uniform(0, 1)
    output_2["w2"] = random.uniform(0, 1)
    output_2["w3"] = random.uniform(0, 1)
    output_2["bias"] = random.uniform(0, 1)


def print_network():
    print("Hidden Layer 1 Weights:")
    print(hidden_1)
    print("\nHidden Layer 2 Weights:")
    print(hidden_2)
    print("\nHidden Layer 3 Weights:")
    print(hidden_3)
    print("\nOutput Layer 1 Weights:")
    print(output_1)
    print("\nOutput Layer 2 Weights:")
    print(output_2)

def train_neural_network(left_sensor, right_sensor, proportional_tuple, neuralnet_tuple):
    #updating new weights

    left_sensor = normalize_sensor(left_sensor)
    right_sensor = normalize_sensor(right_sensor)

    print(left_sensor, right_sensor, "THIS ONE")

    new_h1_w1 = hidden_1["w1"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w1"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w1"]))
    * (h1_out * (1-h1_out)) * left_sensor))
    new_h1_w2 = hidden_1["w2"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w1"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w1"]))
    * (h1_out * (1-h1_out)) * right_sensor))
    new_h1_bias = hidden_1["bias"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w1"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w1"]))
    * (h1_out * (1-h1_out)) * -1))

    new_h2_w1 = hidden_1["w1"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w2"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w2"]))
    * (h1_out * (1-h1_out)) * left_sensor))
    new_h2_w2 = hidden_1["w2"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w2"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w2"]))
    * (h1_out * (1-h1_out)) * right_sensor))
    new_h2_bias = hidden_1["bias"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w2"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w2"]))
    * (h1_out * (1-h1_out)) * -1))

    new_h3_w1 = hidden_1["w1"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w3"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w3"]))
    * (h1_out * (1-h1_out)) * left_sensor))
    new_h3_w2 = hidden_1["w2"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w3"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w3"]))
    * (h1_out * (1-h1_out)) * right_sensor))
    new_h3_bias = hidden_1["bias"] - (LEARN_RATE * 
    ((((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* output_1["w3"])
    + ((neuralnet_tuple[1] - proportional_tuple[1])* (neuralnet_tuple[1] * (1 - neuralnet_tuple[1]))* output_2["w3"]))
    * (h1_out * (1-h1_out)) * -1))

    new_o1_w1 = output_1["w1"] - (LEARN_RATE * 
    ((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* h1_out))
    new_o1_w2 = output_1["w2"] - (LEARN_RATE * 
    ((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* h2_out))
    new_o1_w3 = output_1["w3"] - (LEARN_RATE * 
    ((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* h3_out))
    new_o1_bias = output_1["bias"] - (LEARN_RATE * 
    ((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* -1))

    new_o2_w1 = output_2["w1"] - (LEARN_RATE * 
    ((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* h1_out))
    new_o2_w2 = output_2["w2"] - (LEARN_RATE * 
    ((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* h2_out))
    new_o2_w3 = output_2["w3"] - (LEARN_RATE * 
    ((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* h3_out))
    new_o2_bias = output_2["bias"] - (LEARN_RATE * 
    ((neuralnet_tuple[0] - proportional_tuple[0])* (neuralnet_tuple[0] * (1 - neuralnet_tuple[0]))* -1))


    #update
    hidden_1["w1"] = new_h1_w1
    hidden_1["w2"] = new_h1_w2
    hidden_1["bias"] = new_h1_bias

    print("new w1: ", hidden_1["w1"])


    hidden_2["w1"] =  new_h2_w1
    hidden_2["w2"] =  new_h2_w2
    hidden_2["bias"] =  new_h2_bias

    hidden_3["w1"] = new_h3_w1
    hidden_3["w2"] = new_h3_w2
    hidden_3["bias"] = new_h3_bias

    output_1["w1"] = new_o1_w1
    output_1["w2"] = new_o1_w2
    output_1["w3"] = new_o1_w3
    output_1["bias"] = new_o1_bias

    output_2["w1"] = new_o2_w1
    output_2["w2"] = new_o2_w2
    output_2["w3"] = new_o2_w3
    output_2["bias"] = new_o2_bias

    print("updated")
    print_network()


def denormalize_motor(speed):
    return float(speed*100)

def normalize_sensor(sensor):
    return float(sensor/1000)

def compute_neuralnet(left_sensor: float, right_sensor: float):

    left_sensor = normalize_sensor(left_sensor)
    right_sensor = normalize_sensor(right_sensor)
    print("left " , left_sensor)
    print("right " , right_sensor)

    h1_net = ((left_sensor * hidden_1['w1']) + (right_sensor* hidden_1['w2'])) - hidden_1['bias']
    h2_net = ((left_sensor * hidden_2['w1']) + (right_sensor* hidden_2['w2'])) - hidden_2['bias']
    h3_net = ((left_sensor * hidden_3['w1']) + (right_sensor* hidden_3['w2'])) - hidden_3['bias']

    print("h1_net: ", h1_net)
    print("h2_net: ", h2_net)
    print("h3_net: ", h3_net)


    h1_out = 1/(1 + math.exp(-(h1_net)))
    h2_out = 1/(1 + math.exp(-(h2_net)))
    h3_out = 1/(1 + math.exp(-(h3_net)))  


    print("h1_out: ", h1_out)
    print("h2_out: ", h2_out)
    print("h3_out: ", h3_out)


    o1_net = ((h1_out * output_1['w1']) + (h2_out * output_1['w2']) + (h3_out *output_1['w3'])) - output_1['bias']
    o2_net = ((h1_out * output_2['w1']) + (h2_out * output_2['w2']) + (h3_out *output_2['w3'])) - output_2['bias']

    o1_out = 1/(1 + math.exp(-(o1_net)))
    o2_out = 1/(1 + math.exp(-(o2_net)))
    
    return (o1_out, o2_out)


def compute_proportional(left_sensor: float, right_sensor: float):
    global prev_error
    #derivative = add_to_array_error(error)

    #left_sensor = normalize_sensor(left_sensor)
    #right_sensor = normalize_sensor(right_sensor)
    error = left_sensor - right_sensor


    left_motor_speed = 60 + ((kp) * error)# + (derivative * 0.03) + ((prev_error + error)*0.001)
    right_motor_speed = 60 - ((kp) * error)# - (derivative * 0.03) - ((prev_error + error)*0.001)

    motor(0, left_motor_speed)
    motor(1, right_motor_speed)
    #prev_error = error
    return (left_motor_speed/100, right_motor_speed/100)


def keyboard_input():
    left = 0
    right = 0
    if keyboard.is_pressed('d'):  # if key 'd' is pressed 
        motor(0, 90)
        motor(1, -90)
    elif keyboard.is_pressed('a'):
        motor(0, -90)
        motor(1, 90)
    elif keyboard.is_pressed('w'):
        motor(0, 90)
        motor(1, 90)
    elif keyboard.is_pressed('s'):
        motor(0, -90)
        motor(1, -90)
    
    return (left_ground_sensor.getValue(), right_ground_sensor.getValue())

######################################################################

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# enable the drive motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# enable ground color sensors
left_ground_sensor = robot.getDevice('gs0')
left_ground_sensor.enable(timestep)

middle_ground_sensor = robot.getDevice('gs1')
middle_ground_sensor.enable(timestep)

right_ground_sensor = robot.getDevice('gs2')
right_ground_sensor.enable(timestep)

right_distance_sensor = robot.getDevice('ps2') #IR sensor pointing to the right
right_distance_sensor.enable(timestep)

# initialize encoders
encoders = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoders.append(robot.getDevice(encoderNames[i]))
    encoders[i].enable(timestep)

count = 0
# Main loop:
# - perform simulation steps until Webots stops the controller
network_init()
print("iitied")
print_network()

while robot.step(timestep) != -1:
    # left_motor.setVelocity(1.0) # set the left motor (radians/second)
    # right_motor.setVelocity(1.0) # set the right motor (radians/second) 

    print("left" , left_ground_sensor.getValue())
    print("mid" , middle_ground_sensor.getValue())
    print("right" , right_ground_sensor.getValue())
    print(right_distance_sensor.getValue())

    new_encoder_values = [encoder.getValue() for encoder in encoders]

    if (count < 500):
        print("Training " + str(count))
        #cp_tuple = compute_proportional(left_ground_sensor.getValue(), right_ground_sensor.getValue())
        keyboard_tuple = keyboard_input()
        cp_tuple = compute_proportional(keyboard_tuple[0], keyboard_tuple[1])

        print("Proportional " + str(cp_tuple[0]) + str(cp_tuple[1]))

        nn_tuple = compute_neuralnet(left_ground_sensor.getValue(), right_ground_sensor.getValue())

        print("Neural Net w " + str(nn_tuple[0]) + str(nn_tuple[1]))

        train_neural_network(left_ground_sensor.getValue(), right_ground_sensor.getValue(), cp_tuple, nn_tuple)
        count += 1
    else:
        motor_tuple = compute_neuralnet(left_ground_sensor.getValue(), right_ground_sensor.getValue())
        motor(0, motor_tuple[0])
        motor(1, motor_tuple[1])
        print("Inference: ", motor_tuple)



    #print(new_encoder_values)
    print('-------------------------')
    
    # call robot.getTime() to get the current simulation time in seconds



