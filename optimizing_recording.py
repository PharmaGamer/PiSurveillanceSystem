import os
import time
from datetime import datetime
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import cv2
import numpy as np
import RPi.GPIO as GPIO

# PIR Sensor GPIO pin
PIR_SENSOR_PIN = 23

# Initialize GPIO for PIR sensor
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_SENSOR_PIN, GPIO.IN)

# Initialize the camera (moved inside record_video)
picam2 = None  # Initialize to None

# Define output directory for videos
output_directory = "/var/www/html/videos"
os.makedirs(output_directory, exist_ok=True)

# ... (detect_motion function remains the same)

def record_video():
    global picam2  # Access the global picam2 variable

    # Initialize the camera ONLY when needed
    if picam2 is None:
        picam2 = Picamera2()
        video_config = picam2.create_video_configuration(main={"size": (640, 480)})
        picam2.configure(video_config)
        encoder = H264Encoder(10000000)
        picam2.start() # Start the camera

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    video_filename = os.path.join(output_directory, f"motion_video_{timestamp}.h264")

    print(f"Recording started: {video_filename}")
    picam2.start_recording(encoder, video_filename)

    start_time = time.monotonic()
    prev_frame = picam2.capture_array()

    motion_detected = False
    last_motion_time = time.monotonic()

    while True: # ... (rest of the recording logic remains the same)

    picam2.stop_recording()
    print(f"Recording stopped: {video_filename}")

    # Close the camera immediately after recording
    picam2.close()
    picam2 = None  # Reset picam2 to None
    time.sleep(2) # Give the camera some time to fully shut down.
    

def motion_detected_callback(channel):
    print("Motion detected!")
    record_video()
    print("Waiting for the PIR sensor to stabilize...")
    time.sleep(2)  # Debounce delay

def main():
    print("Motion detection started. Waiting for motion...")
    try:
        GPIO.add_event_detect(PIR_SENSOR_PIN, GPIO.FALLING, callback=motion_detected_callback, bouncetime=200)

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
