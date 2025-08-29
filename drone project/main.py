import threading
import time

from autonomous_drone import AutonomousDrone
from face_detection import FaceDetection
from gps_tracker import GPSTracker
from ir_obstacle import IRObstacle
from pixhawk_stability import PixhawkStability
from ultrasonic_sensors import UltrasonicSensors

# Initialize each module
drone = AutonomousDrone()
face_detector = FaceDetection()
gps = GPSTracker()
ir_sensor = IRObstacle()
stability = PixhawkStability()
ultrasonic = UltrasonicSensors()

# Function to run GPS tracking
def gps_tracking():
    while True:
        gps.track()
        time.sleep(1)

# Function to run obstacle detection
def obstacle_detection():
    while True:
        ir_sensor.detect()
        ultrasonic.scan()
        time.sleep(0.5)

# Function to maintain drone stability
def maintain_stability():
    while True:
        stability.check()
        time.sleep(0.5)

# Function for face detection
def detect_face():
    while True:
        face_detector.run()
        time.sleep(1)

# Running all processes using threads
if __name__ == "__main__":
    threading.Thread(target=gps_tracking, daemon=True).start()
    threading.Thread(target=obstacle_detection, daemon=True).start()
    threading.Thread(target=maintain_stability, daemon=True).start()
    threading.Thread(target=detect_face, daemon=True).start()

    # Main drone control loop
    while True:
        drone.run()
        time.sleep(1)
