# Key Changes for Rerouting:
# Rerouting:

# The drone will move to the right (or another direction) if an obstacle is detected.

# The drone will hold for 5 seconds before returning to the mission path.

# Directional Movement:

# The reroute will involve moving the drone to a safe location and then returning to the mission.

# Obstacle Detection:

# If the distance is below 30 cm, the drone will trigger rerouting.

# Next Steps:
# Test this logic: Run it in a simulation or on your hardware to ensure the movement works smoothly.

# Adjust path behavior: Depending on your setup, you can modify how the drone moves to avoid obstacles (e.g., rotating instead of moving in a straight line).


import asyncio
import statistics
import time

import RPi.GPIO as GPIO
import serial
from mavsdk import System

# Sensor Pins for Ultrasonic Sensors
SENSORS = {
    "Front": {"TRIG": 5, "ECHO": 6},
    "Back": {"TRIG": 12, "ECHO": 16},
    "Left": {"TRIG": 19, "ECHO": 20},
    "Right": {"TRIG": 13, "ECHO": 26},
    "Top": {"TRIG": 21, "ECHO": 25},
    "Bottom": {"TRIG": 23, "ECHO": 24}
}

# IR and TCRT5000 sensor pins
IR_SENSOR = 17
TCRT_SENSOR = 27

# LiDAR setup
LIDAR_SERIAL_PORT = "/dev/serial0"
LIDAR_BAUDRATE = 115200

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_SENSOR, GPIO.IN)
GPIO.setup(TCRT_SENSOR, GPIO.IN)

for sensor in SENSORS.values():
    GPIO.setup(sensor["TRIG"], GPIO.OUT)
    GPIO.setup(sensor["ECHO"], GPIO.IN)

# Initialize LiDAR
try:
    lidar = serial.Serial(LIDAR_SERIAL_PORT, LIDAR_BAUDRATE, timeout=1)
except Exception as e:
    print("Error initializing LiDAR:", e)
    lidar = None

# Distance Measurement Function for Ultrasonic
def get_distance(trig, echo, timeout=0.1):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    
    start_time = time.time()
    timeout_start = time.time()
    while GPIO.input(echo) == 0:
        start_time = time.time()
        if time.time() - timeout_start > timeout:
            return None
    
    end_time = time.time()
    timeout_start = time.time()
    while GPIO.input(echo) == 1:
        end_time = time.time()
        if time.time() - timeout_start > timeout:
            return None
    
    duration = end_time - start_time
    distance = (duration * 34300) / 2
    return round(distance, 2)

# Get LiDAR Distance
def get_lidar_distance():
    if lidar:
        lidar.write(bytearray([0x57, 0x00, 0x02, 0x00, 0x00, 0x02, 0x58]))
        response = lidar.read(9)
        if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            distance = response[2] + (response[3] << 8)
            return distance
    return None

# Reroute Logic - Safe Directional Movement
async def reroute(drone):
    print("Obstacle detected! Initiating reroute.")
    
    # Move in a safe direction (e.g., move to the right by 2 meters)
    await drone.action.goto_location(0.0, 0.0, 10.0, 0)  # Example: move to a safe point
    await asyncio.sleep(5)  # Wait for 5 seconds to clear obstacle

    print("Rerouting complete, returning to path.")
    # Resume the original mission or target location
    await drone.action.goto_location(0.0, 0.0, 10.0, 0)  # Adjust as needed (e.g., return to last known position)

# Drone Mission and Autonomous Flight Control
async def drone_mission():
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Position estimate ready.")
            break
    
    async for home in drone.telemetry.home():
        home_location = home
        print(f"Home position: {home_location.latitude}, {home_location.longitude}")
        break

    try:
        height = float(input("Enter the height (m) for the drone to fly: "))
    except ValueError:
        print("Invalid input. Exiting.")
        return

    print("Arming drone...")
    await drone.action.arm()

    print(f"Taking off and reaching {height} meters...")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    print("Reaching target altitude...")
    await drone.action.goto_location(home_location.latitude, home_location.longitude, height, 0)
    await asyncio.sleep(5)

    print("Hovering for 2 seconds...")
    await asyncio.sleep(2)
    
    print("Returning to launch (RTL)...")
    await drone.action.return_to_launch()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed successfully at home location.")
            break

# Failsafe Monitoring
async def failsafe_monitor(drone):
    async for status in drone.telemetry.status_text():
        if "Fail" in status.text or "Signal lost" in status.text:
            print("Failsafe triggered! Initiating RTL...")
            await drone.action.return_to_launch()
            break

# Monitoring Sensors (Ultrasonic, IR, TCRT5000, LiDAR)
async def monitor_sensors(drone):
    while True:
        distances = {name: get_distance(sensor["TRIG"], sensor["ECHO"]) for name, sensor in SENSORS.items()}
        ir_status = GPIO.input(IR_SENSOR)
        tcrt_status = GPIO.input(TCRT_SENSOR)
        lidar_distance = get_lidar_distance()

        print("\n--- SENSOR DATA ---")
        for sensor, distance in distances.items():
            if distance is not None:
                print(f"{sensor}: {distance} cm")
            else:
                print(f"{sensor}: No reading")

        print(f"IR Sensor Detected Object: {'Yes' if ir_status == 0 else 'No'}")
        print(f"TCRT5000 Sensor Detected Object: {'Yes' if tcrt_status == 0 else 'No'}")
        print(f"LiDAR Distance: {lidar_distance} cm" if lidar_distance else "LiDAR: No reading")

        # Obstacle avoidance logic and rerouting
        for sensor, distance in distances.items():
            if distance and distance < 30:
                print(f"⚠ WARNING: Object too close in {sensor} direction! ({distance} cm)")
                await reroute(drone)  # Trigger rerouting logic

        if lidar_distance and lidar_distance < 30:
            print(f"⚠ WARNING: Object too close in LiDAR direction! ({lidar_distance} cm)")
            await reroute(drone)  # Trigger rerouting logic

        await asyncio.sleep(1)

# Main function (Drone control and monitoring)
async def main():
    drone = System()
    await drone.connect(system_address="udp://:14550")
    serial_drone = System()
    await serial_drone.connect(system_address="serial:///dev/ttyS1:57600")

    await asyncio.gather(drone_mission(), failsafe_monitor(drone), failsafe_monitor(serial_drone), monitor_sensors(drone))

# Start program
if __name__ == "__main__":
    asyncio.run(main())

