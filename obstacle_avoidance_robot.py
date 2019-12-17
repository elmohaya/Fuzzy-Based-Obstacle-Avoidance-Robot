#!/usr/bin/env python
# coding: utf-8


from time import sleep
import RPi.GPIO as GPIO
import time
import pandas as pd


#reading the csv file

df = pd.read_csv('obstacle_robot_data.csv')

#Import time library
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


GPIO.setup(21, GPIO.IN)
GPIO.setup(26, GPIO.IN)
GPIO.setup(13, GPIO.IN)

GPIO.setup(20, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

left_motor = GPIO.PWM(17, 1000)
right_motor = GPIO.PWM(27, 1000)
left_motor.start(65)
right_motor.start(65)

left_motor_signal_1 = 65
right_motor_signal_1 = 65


trigs = [20, 19, 6]
echos = [21, 26, 13]
distance = [0.0, 0.0, 0.0]






# [System]
# Name='Obstacle_Avoidance_Robot'
# Type='mamdani'
# Version=2.0
# NumInputs=3
# NumOutputs=2
# NumRules=7
# AndMethod='min'
# OrMethod='max'
# ImpMethod='prod'
# AggMethod='max'
# DefuzzMethod='COS'

# [Input1]
# Name='left_sensor'
# Range=[2 400] cm
# NumMFs=2

# [Input2]
# Name='med_sensor'
# Range=[2 400] cm
# NumMFs=2

# [Input3]
# Name='right_sensor'
# Range=[2 400] cm
# NumMFs=2

Input_Close=[0, 0, 15, 30]
Input_Far=[20 ,200 ,400 ,400]

# [Output1]
# Name='left_motor'
# Range=[50 100]
# NumMFs=3

# #[Output2]
# Name='right_motor'
# Range=[50 100]
# NumMFs=3

Output_Slow=[20 ,20 ,50]
Output_Med=[45 ,60 ,80]
Output_Fast=[75 ,100 ,100]


# In[28]:


# Implication and Inference

def calc_input_close (x):

    # the objective of this function is find
    # the Degree of Fullfillment of input low

    if x < Input_Close[0]:
        result = 0.0
    elif x<= Input_Close[2]:
        result = 1.0
    elif x< Input_Close[3]:
        result = (-1.0/(Input_Close[3]-Input_Close[2]))*(x-Input_Close[3])
    else:
        result = 0.0

    return result

#####################################

def calc_input_far (x):

    # the objective of this function is find
    # the Degree of Fullfillment of input high

    if x < Input_Far[0]:
        result = 0.0
    elif x<= Input_Far[1]:
        result = (1.0/(Input_Far[1]-Input_Far[0]))*(x-Input_Far[0])
    elif x<= Input_Far[2]:
        result = 1.0
    else:
        result = 0.0

    return result

# In[52]:
while True:

    distance = [0.0, 0.0, 0.0]
    initial_time = time.time()

    for j in range(3):
        sum1= 0.0

        for i in range(10):


            GPIO.output(trigs[j], False)
            time.sleep(0.000004)
            GPIO.output(trigs[j], True)
            time.sleep(0.00001)
            GPIO.output(trigs[j], False)


            pulse_start = time.time()
            activate = 0
            timer = time.time()
            while True:

                if GPIO.input(echos[j])==0:
                    if activate == 1 or timer > 0.01:
                        break
                    else:
                        pulse_start = time.time()


                elif GPIO.input(echos[j])==1:
                    pulse_end = time.time()
                    activate = 1

                timer = time.time() - timer


            pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            dist = pulse_duration * 17150
            sum1 = sum1 + dist


        print(' ')
        distance[j] = round(sum1 / 10.0, 2 )


    left_sensor = distance[0]
    right_sensor= distance[2]
    med_sensor = distance[1]


# Evaluating the rules


# R1: if (left_Sensor is Close) & (med_sensor is Close) & (right_sensor is Far) then (left_motor is Fast)(right_motor is Slow):

    DOF1 = min(calc_input_close(left_sensor), calc_input_close(med_sensor), calc_input_far(right_sensor))
    infer_r1_out1 = list()
    infer_r1_out2 = list()

# Sampling the area of the inference
    for i in range(Output_Fast[0], Output_Fast[1]+1, 1):
        infer_r1_out1.append([i, DOF1*(1.0/(Output_Fast[1]-Output_Fast[0]))*(i-Output_Fast[0])])
    for i in range(Output_Slow[1], Output_Slow[2]+1, 1):
        infer_r1_out2.append([i, DOF1*(-1.0/(Output_Slow[2]-Output_Slow[1]))*(i-Output_Slow[2])])

############################################################################

# R2: if (left_Sensor is Close) & (med_sensor is Far) & (right_sensor is Close) then (left_motor is Med)(right_motor is Med):

    DOF2 = min(calc_input_close(left_sensor), calc_input_far(med_sensor), calc_input_close(right_sensor))
    infer_r2_out1 = list()


# Sampling the area of the inference
    for i in range(Output_Med[0], Output_Med[1]+1, 1):
        infer_r2_out1.append([i, DOF2*(1.0/(Output_Med[1]-Output_Med[0]))*(i-Output_Med[0])])
    for i in range(Output_Med[1], Output_Med[2]+1, 1):
        infer_r2_out1.append([i, DOF2*(-1.0/(Output_Med[2]-Output_Med[1]))*(i-Output_Med[2])])
    infer_r2_out2 = infer_r2_out1
############################################################################

# R3: if (left_Sensor is Far) & (med_sensor is Close) & (right_sensor is Close) then (left_motor is Slow)(right_motor is Fast):

    DOF3 = min(calc_input_far(left_sensor), calc_input_close(med_sensor), calc_input_close(right_sensor))
    infer_r3_out1 = list()
    infer_r3_out2 = list()

# Sampling the area of the inference
    for i in range(Output_Fast[0], Output_Fast[1]+1, 1):
        infer_r3_out2.append([i, DOF3*(1.0/(Output_Fast[1]-Output_Fast[0]))*(i-Output_Fast[0])])
    for i in range(Output_Slow[1], Output_Slow[2]+1, 1):
        infer_r3_out1.append([i, DOF3*(-1.0/(Output_Slow[2]-Output_Slow[1]))*(i-Output_Slow[2])])


############################################################################

# R4: if (left_Sensor is close) & (med_sensor is far) & (right_sensor is far) then (left_motor is Fast)(right_motor is Slow):

    DOF4 = min(calc_input_close(left_sensor), calc_input_far(med_sensor), calc_input_far(right_sensor))
    infer_r4_out1 = list()
    infer_r4_out2 = list()

# Sampling the area of the inference
    for i in range(Output_Fast[0], Output_Fast[1]+1, 1):
        infer_r4_out1.append([i, DOF4*(1.0/(Output_Fast[1]-Output_Fast[0]))*(i-Output_Fast[0])])
    for i in range(Output_Slow[1], Output_Slow[2]+1, 1):
        infer_r4_out2.append([i, DOF4*(-1.0/(Output_Slow[2]-Output_Slow[1]))*(i-Output_Slow[2])])
############################################################################

# R5: if (left_Sensor is Far) & (med_sensor is Close) & (right_sensor is Far) then (left_motor is Slow)(right_motor is Fast):

    DOF5 = min(calc_input_far(left_sensor), calc_input_close(med_sensor), calc_input_far(right_sensor))
    infer_r5_out1 = list()
    infer_r5_out2 = list()

# Sampling the area of the inference
    for i in range(Output_Fast[0], Output_Fast[1]+1, 1):
        infer_r5_out2.append([i, DOF5*(1.0/(Output_Fast[1]-Output_Fast[0]))*(i-Output_Fast[0])])
    for i in range(Output_Slow[1], Output_Slow[2]+1, 1):
        infer_r5_out1.append([i, DOF5*(-1.0/(Output_Slow[2]-Output_Slow[1]))*(i-Output_Slow[2])])

    ############################################################################

# R6: if (left_Sensor is Far) & (med_sensor is Far) & (right_sensor is Close) then (left_motor is Slow)(right_motor is Fast):

    DOF6 = min(calc_input_far(left_sensor), calc_input_far(med_sensor), calc_input_close(right_sensor))
    infer_r6_out1 = list()
    infer_r6_out2 = list()

# Sampling the area of the inference
    for i in range(Output_Fast[0], Output_Fast[1]+1, 1):
        infer_r6_out2.append([i, DOF6*(1.0/(Output_Fast[1]-Output_Fast[0]))*(i-Output_Fast[0])])
    for i in range(Output_Slow[1], Output_Slow[2]+1, 1):
        infer_r6_out1.append([i, DOF6*(-1.0/(Output_Slow[2]-Output_Slow[1]))*(i-Output_Slow[2])])

    ############################################################################

# R7: if (left_Sensor is Far) & (med_sensor is Far) & (right_sensor is Far) then (left_motor is Fast)(right_motor is Fast):

    DOF7 = min(calc_input_far(left_sensor), calc_input_far(med_sensor), calc_input_far(right_sensor))
    infer_r7_out1 = list()


# Sampling the area of the inference
    for i in range(Output_Fast[0], Output_Fast[1]+1, 1):
        infer_r7_out1.append([i, DOF7*(1.0/(Output_Fast[1]-Output_Fast[0]))*(i-Output_Fast[0])])
    infer_r7_out2 = infer_r7_out1



    print(DOF1, DOF2, DOF3, DOF4, DOF5, DOF6, DOF7)


    infer_out1 = infer_r1_out1 + infer_r2_out1 + infer_r3_out1 + infer_r4_out1 + infer_r5_out1 + infer_r6_out1 + infer_r7_out1
    infer_out2 = infer_r1_out2 + infer_r2_out2 + infer_r3_out2 + infer_r4_out2 + infer_r5_out2 + infer_r6_out2 + infer_r7_out2

#print(infer_out1)
#print('  ')
#print(infer_out2)
#print('  ')
#print(DOF1, DOF2, DOF3, DOF4, DOF5)


# In[53]:


# Defuzzification Code

    num1 = 0.0
    den1 = 0.0

    num2 = 0.0
    den2 = 0.0


    if (DOF1 != 0) or (DOF2 != 0) or (DOF3 != 0) or (DOF4 != 0) or (DOF5 != 0) or (DOF6 != 0) or (DOF7 != 0):

        for i in range(len(infer_out1)):
            num1 = num1 + infer_out1[i][0]*infer_out1[i][1]
            den1 = den1 + infer_out1[i][1]

        for i in range(len(infer_out2)):
            num2 = num2 + infer_out2[i][0]*infer_out2[i][1]
            den2 = den2 + infer_out2[i][1]

        left_motor_signal = num1/den1
        right_motor_signal = num2/den2
        left_motor_signal_1 = left_motor_signal
        right_motor_signal_1 = right_motor_signal
    else:
        left_motor_signal= left_motor_signal_1
        right_motor_signal= right_motor_signal_1

    left_motor.ChangeDutyCycle(left_motor_signal)
    right_motor.ChangeDutyCycle(right_motor_signal)

    sampling_time = time.time()-initial_time
    s = pd.Series([left_sensor, med_sensor, right_sensor, sampling_time, 3.3*left_motor_signal/100, 3.3*right_motor_signal/100],
                  index = ['left_sensor', 'med_sensor', 'right_sensor', 'sampling_time', 'left_motor', 'right_motor'])
    df = df.append(s, ignore_index =True)
    df.to_csv('obstacle_robot_data.csv', index = False)

    print(' ')
    print(left_sensor, med_sensor , right_sensor, left_motor_signal, right_motor_signal)
    print('   ')
