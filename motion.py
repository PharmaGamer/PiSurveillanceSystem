import RPi.GPIO as GPIO
import time

# Set up the GPIO pin for PIR sensor
PIR_PIN = 23  # GPIO pin where PIR sensor is connected
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_PIN, GPIO.IN)

try:
    while True:
        if GPIO.input(PIR_PIN) == GPIO.HIGH:
            print("PIR Sensor is HIGH (motion detected)")
        else:
            print("PIR Sensor is LOW (no motion)")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    GPIO.cleanup()

