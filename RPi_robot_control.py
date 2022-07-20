import sys
import time
import RPi.GPIO as GPIO

def setup():
    mode = GPIO.getmode()

    GPIO.cleanup()

    global Forward, Backward, Forward2, Backward2, pwm, pwm2
    Forward = 26
    Backward = 20
    Forward2 = 19
    Backward2 = 16
    sleeptime = 1

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Forward, GPIO.OUT)
    GPIO.setup(Backward, GPIO.OUT)
    GPIO.setup(Forward2, GPIO.OUT)
    GPIO.setup(Backward2, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)

    pwm = GPIO.PWM(18, 1000)
    pwm.start(20)
    pwm2 = GPIO.PWM(13, 1000)
    pwm2.start(20)

def turn_right(command):
    # turns right
    if command == 0:
        pass
    else:
        GPIO.output(Forward2, GPIO.HIGH)
        GPIO.output(Forward, GPIO.HIGH)
        pwm.start(100)
        pwm2.start(100)
        pwm.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(100)
        #time.sleep(1)

def turn_left(command):
    #Turns left
    if command == 0:
        pass
    else:
        GPIO.output(Forward2, GPIO.HIGH)
        GPIO.output(Forward, GPIO.HIGH)
        pwm.start(100)
        pwm2.start(100)
        pwm.ChangeDutyCycle(100)
        pwm2.ChangeDutyCycle(0)

def go_straight_with_speed(speed):
    #speed goes from 0(slowest) to 100 (fastest)
    pwm.start(speed)
    pwm2.start(speed)
    GPIO.output(Forward2, GPIO.HIGH)
    GPIO.output(Forward, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def stop():
    pwm.stop()
    pwm2.stop()
    GPIO.output(Forward, GPIO.LOW)
    GPIO.output(Forward2, GPIO.LOW)
    GPIO.cleanup()

