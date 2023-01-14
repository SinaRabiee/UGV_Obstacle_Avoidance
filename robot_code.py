import RPi.GPIO as GPIO          

import numpy as np

import smbus

import time



srf02_right_addr = 0x70

srf02_left_addr = 0x75

pins=np.array([[23,24,22],[17,27,18],[21,20,16],[26,13,19]]) #motors pin(1,2,3,4)

bus = smbus.SMBus(1)

time.sleep(2) #wait here to avoid 121 IO Error



class robot:

    def __init__(self):

        GPIO.setmode(GPIO.BCM)

        self.sensor_left = 0

        self.sensor_right = 0

        self.motor_BL = 100

        self.motor_BR = 100

        self.motor_FL = 100

        self.motor_FR = 100

        self.ctr = 60



    def read_srf02(self,address):

        bus.write_i2c_block_data(address,0x00,[0x51])

        # time.sleep(1)

        data = bus.read_i2c_block_data(address,0x00,4)

        for b in data:

            # result1 = result1 * 256 + int(b)

            distance = (data[2]<<8 | data[3])

        return distance



    def pin_starter(self,in1, in2, pwm_pin):

        GPIO.setup(in1,GPIO.OUT)

        GPIO.setup(in2,GPIO.OUT)

        GPIO.output(in1,GPIO.LOW)

        GPIO.output(in2,GPIO.LOW)

        GPIO.setup(pwm_pin,GPIO.OUT)

        print("pins started")



    def robot_starter(self):

        for i in range(pins.shape[0]):

            self.pin_starter(int(pins[i,0]), int(pins[i,1]),int(pins[i,2]))        

        self.pwm1 = GPIO.PWM(int(pins[0,2]),1000)

        self.pwm1.start(0)

        self.pwm2 = GPIO.PWM(int(pins[1,2]),1000)

        self.pwm2.start(0)

        self.pwm3 = GPIO.PWM(int(pins[2,2]),1000)

        self.pwm3.start(0)

        self.pwm4 = GPIO.PWM(int(pins[3,2]),1000)

        self.pwm4.start(0)

        print("robot initialized")

        return 0



    def motor_run(self,motor_number, speed, dir):

        # GPIO.PWM(int(pins[motor_number,2]),1000).ChangeDutyCycle(speed)



        if(dir):

            GPIO.output(int(pins[motor_number,0]),GPIO.HIGH)

            GPIO.output(int(pins[motor_number,1]),GPIO.LOW)

        else:

            GPIO.output(int(pins[motor_number,1]),GPIO.HIGH)

            GPIO.output(int(pins[motor_number,0]),GPIO.LOW)

        return 0



    def robot_run(self,speed1, speed2, speed3, speed4):

        if (speed1 > 0):

            dir1 = 1

        else:

            dir1 = 0

            speed1 = -speed1

        self.motor_run(0, speed1, dir1)

        self.pwm1.ChangeDutyCycle(speed1)

        if (speed2 > 0):

            dir2 = 1

        else:

            dir2 = 0

            speed2 = -speed2

        self.motor_run(1, speed2, dir2)

        self.pwm2.ChangeDutyCycle(speed2)

        if (speed3 > 0):

            dir3 = 1

        else:

            dir3 = 0

            speed3 = -speed3

        self.motor_run(2, speed3, dir3)

        self.pwm3.ChangeDutyCycle(speed3)

        if (speed4 > 0):

            dir4 = 1

        else:

            dir4 = 0

            speed4 = -speed4

        self.motor_run(3, speed4, dir4)

        self.pwm4.ChangeDutyCycle(speed4)

        return 0



    def robot_stop(self):

        for i in range(pins.shape[0]):

            GPIO.output(int(pins[i,0]),GPIO.HIGH)

            GPIO.output(int(pins[i,0]),GPIO.LOW)

        return 0

    def update(self):

        self.sensor_left = self.read_srf02(srf02_left_addr)

        self.sensor_right = self.read_srf02(srf02_right_addr)

        self.robot_run(self.motor_BL,self.motor_BR,self.motor_FL,self.motor_FR)

robot1 = robot()

robot1.robot_starter()





left_wheel = list()

right_wheel = list()

left_srf = list()

right_srf = list()

last_sensor_left = 0

last_sensor_right = 0

while True:

    

    #print("srf_left:"+str(robot1.sensor_left) + ",srf_right: "+ str(robot1.sensor_right))

    # robot1.robot_run(100,100,100, 100)

    if (robot1.ctr > 20):

        #print("resid") 

        time.sleep(0.1)  

        robot1.motor_BL = robot1.ctr

        robot1.motor_BR = robot1.ctr

        robot1.motor_FL = robot1.ctr

        robot1.motor_FR = robot1.ctr+40

        left_wheel.append(robot1.ctr)

        right_wheel.append(robot1.ctr)

        left_srf.append(robot1.sensor_left)

        right_srf.append(robot1.sensor_right)

        robot1.update()

        robot1.ctr = robot1.ctr -1

    elif (robot1.ctr == 20):

        robot1.ctr = 0

        robot1.robot_run(0,0,0,0)

        f = open("left_srf.txt","w")

        f.write(str(left_srf))

        f.close()

        f = open("right_srf.txt","w")

        f.write(str(right_srf))

        f.close()

        f = open("pwm_left.txt","w")

        f.write(str(left_wheel))

        f.close()

        f = open("pwm_right.txt","w")

        f.write(str(right_wheel))

        f.close()

        




