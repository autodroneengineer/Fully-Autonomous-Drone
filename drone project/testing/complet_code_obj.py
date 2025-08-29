import math
import threading
import time

import RPi.GPIO as GPIO
from dronekit import LocationGlobalRelative, VehicleMode, connect

# === Ultrasonic Sensor Pins ===
ULTRASONIC_PINS = {
    "front": (23, 24),
    "left": (25, 26),
    "right": (27, 22),
    "back": (5, 6),
    "top": (13, 19),
    "bottom": (16, 20),
}

# Setup GPIO
GPIO.setmode(GPIO.BCM)
for trig, echo in ULTRASONIC_PINS.values():
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(False)
    GPIO.setup(echo, GPIO.IN)

# === Function to measure distance ===
def measure_distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.05)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    pulse_start, pulse_end = 0, 0
    timeout = time.time() + 1
    while GPIO.input(echo) == 0 and time.time() < timeout:
        pulse_start = time.time()

    while GPIO.input(echo) == 1 and time.time() < timeout:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # in cm
    return round(distance, 2)

# === Check for obstacle in any direction ===
def is_obstacle_detected(threshold=100):  # cm
    for direction, (trig, echo) in ULTRASONIC_PINS.items():
        distance = measure_distance(trig, echo)
        print(f"{direction.capitalize()} Distance: {distance} cm")
        if distance < threshold:
            print(f"Obstacle detected {direction.upper()}! ({distance} cm)")
            return True
    return False

# === Connect to vehicle ===
print("Connecting to drone...")
vehicle = connect('/dev/ttyS0', wait_ready=True, baud=57600)

# === User Input ===
dest_lat = float(input("Enter destination latitude: "))
dest_lon = float(input("Enter destination longitude: "))
stay_time = int(input("Enter time to stay at destination (in seconds): "))
target_altitude = float(input("Enter flight altitude in meters: "))

# === Takeoff ===
def arm_and_takeoff(aTargetAltitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f} m")
        if alt >= aTargetAltitude * 0.95:
            print("Target altitude reached.")
            break
        time.sleep(1)

# === Distance Between Two GPS Points ===
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlon = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlon * 1.113195e5)**2)

# === Obstacle-Aware Navigation ===
def go_to_location_with_avoidance(destination):
    print("Navigating with obstacle avoidance...")
    vehicle.simple_goto(destination)

    while True:
        current_location = vehicle.location.global_relative_frame
        distance_to_target = get_distance_metres(current_location, destination)
        print(f"Distance to target: {distance_to_target:.2f} m")

        if is_obstacle_detected():
            print("Obstacle detected! Hovering...")
            vehicle.mode = VehicleMode("LOITER")
            time.sleep(3)
            vehicle.mode = VehicleMode("GUIDED")
            vehicle.simple_goto(destination)

        if distance_to_target < 2.0:
            print("Reached destination.")
            break
        time.sleep(2)

# === Safe Landing ===
def safe_land():
    print("Landing safely...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Landing...")
        time.sleep(2)
    print("Drone has landed.")

# === Error Monitoring ===
def monitor_errors():
    try:
        while True:
            if not vehicle.armed or not vehicle.is_armable:
                print("Drone unarmed unexpectedly! Landing...")
                safe_land()
                break
            time.sleep(1)
    except:
        print("System error occurred. Landing...")
        safe_land()

# === MAIN Mission ===
try:
    threading.Thread(target=monitor_errors, daemon=True).start()
    arm_and_takeoff(target_altitude)

    destination = LocationGlobalRelative(dest_lat, dest_lon, target_altitude)
    go_to_location_with_avoidance(destination)

    print(f"Holding at destination for {stay_time} seconds...")
    time.sleep(stay_time)

    print("Returning to launch...")
    vehicle.mode = VehicleMode("RTL")

    while vehicle.armed:
        print("Returning home...")
        time.sleep(3)

    print("Mission complete.")

except Exception as e:
    print(f"Mission error: {e}")
    safe_land()

finally:
    GPIO.cleanup()
    vehicle.close()
