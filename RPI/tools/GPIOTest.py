from gpiozero import Button
from time import sleep

limit = Button(17, pull_up=True)   # Using NO wiring

try:
    while True:
        if limit.is_pressed:
            print("Limit switch PRESSED")
        else:
            print("Limit switch RELEASED")
        sleep(0.1)
except KeyboardInterrupt:
    pass
