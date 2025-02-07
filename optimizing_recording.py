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

def detect_motion(frame1, frame2, threshold=30):
    """Detects motion by comparing two frames."""
    diff = cv2.absdiff(frame1, frame2)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)
    motion = np.sum(thresh) > threshold
    return motion

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

    while True:
        current_frame = picam2.capture_array()
        motion_detected = detect_motion(prev_frame, current_frame)
        prev_frame = current_frame

        if motion_detected:
            print(f"Motion detected.{last_motion_time}")
            last_motion_time = time.monotonic()    # Reset the motion timeout
        elif time.monotonic() - last_motion_time > 10:
            print("No motion detected for 10 seconds. Stopping recording.")
            break

        # Stop recording after 5 minutes
        if time.monotonic() - start_time > 300:
            print("Maximum recording time reached. Stopping recording.")
            break

        # Ensure at least 10 seconds of recording
        if time.monotonic() - start_time > 10 and time.monotonic() - last_motion_time > 10:
            break

        time.sleep(0.1)  # Small delay to reduce CPU usage

    picam2.stop_recording()
    print(f"Recording stopped: {video_filename}")

    # Close the camera immediately after recording
    picam2.close()
    picam2 = None  # Reset picam2 to None
    time.sleep(2) # Give the camera some time to fully shut down.


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
