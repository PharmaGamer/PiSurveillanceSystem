ssssqimport os
import time
from datetime import datetime
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import RPi.GPIO as GPIO

# PIR Sensor GPIO pin
PIR_SENSOR_PIN = 23

# Initialize GPIO for PIR sensor
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_SENSOR_PIN, GPIO.IN)

# Initialize the camera
picam2 = Picamera2()
video_config = picam2.create_video_configuration()
picam2.configure(video_config)
encoder = H264Encoder(10000000)

# Define output directory for videos
output_directory = "/var/www/html/videos"
os.makedirs(output_directory, exist_ok=True)

def record_video():
    """Records a 30-second video when motion is detected."""
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    video_filename = os.path.join(output_directory, f"motion_video_{timestamp}.mp4")

    print(f"Recording started: {video_filename}")
    picam2.start_recording(encoder, video_filename)
    time.sleep(30)  # Record for a minimum of 30 seconds
    picam2.stop_recording()
    print(f"Recording stopped: {video_filename}")

def main():
    """Main loop for motion detection and recording."""
    print("Motion detection started. Waiting for motion...")
    try:
        while True:
            if GPIO.input(PIR_SENSOR_PIN):
                print("Motion detected!")
                record_video()
                print("Waiting for the PIR sensor to stabilize...")
                time.sleep(5)  # Delay to avoid immediate retriggering
            else:
                time.sleep(0.1)  # Small delay to reduce CPU usage
    except KeyboardInterrupt:
        print("Exiting motion detection.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
