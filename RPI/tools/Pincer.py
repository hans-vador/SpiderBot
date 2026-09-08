import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

p = GPIO.PWM(17, 50)
p.start(1)  # sends approx 0° command
time.sleep(3)

p.ChangeDutyCycle(12)  # sends approx 180° command
time.sleep(3)

p.stop()
GPIO.cleanup()
