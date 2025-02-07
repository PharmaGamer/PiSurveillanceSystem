import os
import time
from datetime import datetime
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import cv2
import numpy as np
import RPi.GPIO as GPIO

# ... (other constants and setup)

def record_video():
    # ... (camera initialization and recording logic - same as before)

def motion_detected_callback(channel):
    print("Motion detected!")
    time.sleep(0.05) # Debounce at the start
    if GPIO.input(PIR_SENSOR_PIN) == GPIO.LOW: # Check if it's still low, if not, it was a false trigger
        return

    start_time = time.monotonic()
    while GPIO.input(PIR_SENSOR_PIN) == GPIO.HIGH:  # Check if it's still high
        time.sleep(0.05)  # Small delay
        if time.monotonic() - start_time > 0.2:  # Minimum 200ms trigger time
            record_video()
            print("Waiting for the PIR sensor to stabilize...")
            time.sleep(2)  # Debounce delay
            return # Exit to avoid repeated triggers
    print("False trigger or short event") # If it gets here, it was a short event


def main():
    print("Motion detection started. Waiting for motion...")
    try:
        GPIO.add_event_detect(PIR_SENSOR_PIN, GPIO.RISING, callback=motion_detected_callback, bouncetime=200) # changed to RISING

        while True:
            time.sleep(1)  # Main loop sleeps while waiting for interrupts
    except KeyboardInterrupt:
        print("Exiting motion detection.")
    finally:
        GPIO.cleanup()
        if picam2 is not None:  # Check if camera was initialized
            picam2.close() # Close the camera if it's open


if __name__ == "__main__":
    main()
