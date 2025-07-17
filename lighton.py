import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)

GPIO.output(21, GPIO.HIGH)

try:
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()