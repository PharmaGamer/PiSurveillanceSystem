# PiSurveillanceSystem
Program for RPi Zero 2 W and camera with PIR motion sensor

Requirements:
Raspberry Pi Zero 2 W

Raspberry Pi Camera Module 3 (or any others that you like and are compatible with the RPi Zero 2 W)

HC-SR501 PIR sensor with signal connected to GPIO 23
  
I'm working out how to put this all in a neat and tidy case.

Standard Bookworm Lite OS installation

Packages required:

	python3-picamera2

	lighttpd

		Configured directory listing (dir-listing.activate = "enable") in /etc/lighttpd/lighttpd.conf.

		Hosted videos in /var/www/html/videos.

	python3-rpi.gpio

	ffmpeg

	rclone

Filesystem changes:
sudo chmod -R 777 /var/www/html/videos
