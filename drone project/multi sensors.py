import time

import RPi.GPIO as GPIO
import serial

SENSORS = {
    "Front": {"TRIG": 5, "ECHO": 6},
    "Back": {"TRIG": 12, "ECHO": 16},
    "Left": {"TRIG": 19, "ECHO": 20},
    "Right": {"TRIG": 13, "ECHO": 26},
    "Top": {"TRIG": 21, "ECHO": 25},
    "Bottom": {"TRIG": 23, "ECHO": 24}
}

IR_SENSOR = 17
TCRT_SENSOR = 27
LIDAR_SERIAL_PORT = "/dev/serial0"
LIDAR_BAUDRATE = 115200

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_SENSOR, GPIO.IN)
GPIO.setup(TCRT_SENSOR, GPIO.IN)

for sensor in SENSORS.values():
    GPIO.setup(sensor["TRIG"], GPIO.OUT)
    GPIO.setup(sensor["ECHO"], GPIO.IN)

try:
    lidar = serial.Serial(LIDAR_SERIAL_PORT, LIDAR_BAUDRATE, timeout=1)
except Exception as e:
    print("Error initializing LiDAR:", e)
    lidar = None

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

def get_lidar_distance():
    if lidar:
        lidar.write(bytearray([0x57, 0x00, 0x02, 0x00, 0x00, 0x02, 0x58]))
        response = lidar.read(9)
        if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            distance = response[2] + (response[3] << 8)
            return distance
    return None

try:
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

        for sensor, distance in distances.items():
            if distance and distance < 30:
                print(f"⚠ WARNING: Object too close in {sensor} direction! ({distance} cm)")

        if lidar_distance and lidar_distance < 30:
            print(f"⚠ WARNING: Object too close in LiDAR direction! ({lidar_distance} cm)")

        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting Program...")
    GPIO.cleanup()
    if lidar:
        lidar.close()
