#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import cv2
#Definition of  pin 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Definition of button
key = 8

#Definition of ultrasonic module pin
EchoPin = 0
TrigPin = 1
#0,1/

#Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24
#Definition of servo pin
ServoPin = 23

#Definition of infrared obstacle avoidance module pins
AvoidSensorLeft = 12
AvoidSensorRight = 17

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)


#Motor pins are initialized into output mode
#Key pin is initialized into input mode
#Ultrasonic pin initialization
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft,GPIO.IN)
    GPIO.setup(AvoidSensorRight,GPIO.IN)
    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)
	
#Advance
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#back
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
	
#turn left
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#turn right
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
	
#turn left in place
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#turn right in place
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#brake
def brake():
   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)

#Button detection
def key_scan():
    while GPIO.input(key):
         pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
             time.sleep(0.01)
        while not GPIO.input(key):
	         pass
				
def Distance():
    GPIO.output(TrigPin,GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TrigPin,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin,GPIO.LOW)

    t3 = time.time()

    while not GPIO.input(EchoPin):
        t4 = time.time()
        if (t4 - t3) > 0.03 :
            return -1


    t1 = time.time()
    while GPIO.input(EchoPin):
        t5 = time.time()
        if(t5 - t1) > 0.03 :
            return -1

    t2 = time.time()
    time.sleep(0.01)
    print ("distance is %d " % (((t2 - t1)* 340 / 2) * 100))
    return ((t2 - t1)* 340 / 2) * 100

	
def Distance_test():
    num = 0
    ultrasonic = []
    while num < 5:
            distance = Distance()
            while int(distance) == -1 :
                distance = Distance()
                print("Tdistance is %f"%(distance) )
            while (int(distance) >= 500 or int(distance) == 0) :
                distance = Distance()
                print("Edistance is %f"%(distance) )
            ultrasonic.append(distance)
            num = num + 1
            time.sleep(0.01)
        #print ultrasonic
    distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3])/3
    print("distance is %f"%(distance) ) 
    return distance


#The servo rotates to the specified angle
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)	
		
def servo_color_carstate():
    #red
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)
    back(10,10)
    time.sleep(0.08)
    brake()
	
    servo_appointed_detection(0)
    time.sleep(0.8)
    rightdistance = Distance_test()
  
    servo_appointed_detection(180)
    time.sleep(0.8)
    leftdistance = Distance_test()

    servo_appointed_detection(90)
    time.sleep(0.8)
    frontdistance = Distance_test()
 
    if leftdistance < 30 and rightdistance < 30 and frontdistance < 30:
        #Magenta
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_right(30,30)
        time.sleep(0.58)
    elif leftdistance >= rightdistance:
	#Blue
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_left(30,30)
        time.sleep(0.28)
    elif leftdistance <= rightdistance:
	#Magenta
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_right(30,30)
        time.sleep(0.28)
		
#delay 2s	
time.sleep(2)
# Initialize camera
cap = cv2.VideoCapture(0)  # Use the first USB camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Allow the camera to warm up
time.sleep(0.1)


# Video saving setup
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('ultra.mp4', fourcc, 8, (320, 240))

time.sleep(1)

#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.
try:
    
    init()

    while True:
        ret, frame = cap.read()
        distance = Distance_test()
        if distance > 40:
         #There is obstacle, the indicator light of the infrared obstacle avoidance module is on, and the port level is LOW
         #There is no obstacle, the indicator light of the infrared obstacle avoidance module is off, and the port level is HIGH
            LeftSensorValue  = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)

            if LeftSensorValue == True and RightSensorValue == True :
                run(30,30) 
                run(30,30)         
            elif LeftSensorValue == True and RightSensorValue == False :
                spin_left(30,30)     
                time.sleep(0.002)
            elif RightSensorValue == True and LeftSensorValue == False:
                spin_right(10,10)    
                time.sleep(0.002)				
            elif RightSensorValue == False and LeftSensorValue == False :
                spin_right(10,10)    
                time.sleep(0.002)
                run(10,10)
            GPIO.output(LED_R, GPIO.LOW)
            GPIO.output(LED_G, GPIO.HIGH)
            GPIO.output(LED_B, GPIO.LOW)
        elif 30 <= distance <= 40:
         #There is obstacle, the indicator light of the infrared obstacle avoidance module is on, and the port level is LOW
         #There is no obstacle, the indicator light of the infrared obstacle avoidance module is off, and the port level is HIGH
            LeftSensorValue  = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)

            if LeftSensorValue == True and RightSensorValue == True :
                run(20,20)         
            elif LeftSensorValue == True and RightSensorValue == False :
                spin_left(10,10)     
                time.sleep(0.002)
            elif RightSensorValue == True and LeftSensorValue == False:
                spin_right(10,10)    
                time.sleep(0.002)				
            elif RightSensorValue == False and LeftSensorValue == False :
                spin_right(10,10)    
                time.sleep(0.002)
                run(10,10)
        elif distance < 30:
            servo_color_carstate()
        out.write(frame)
    
    
except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()
