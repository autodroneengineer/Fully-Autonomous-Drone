import math
import time

import rclpy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Imu, NavSatFix

# Ultrasonic sensor pins (same as before)
ULTRASONIC_TRIG_FRONT = 23
ULTRASONIC_ECHO_FRONT = 24
ULTRASONIC_TRIG_LEFT = 25
ULTRASONIC_ECHO_LEFT = 26
ULTRASONIC_TRIG_RIGHT = 27
ULTRASONIC_ECHO_RIGHT = 22
ULTRASONIC_TRIG_BACK = 5
ULTRASONIC_ECHO_BACK = 6
ULTRASONIC_TRIG_TOP = 13
ULTRASONIC_ECHO_TOP = 19
ULTRASONIC_TRIG_BOTTOM = 16
ULTRASONIC_ECHO_BOTTOM = 20

# GPIO Setup
GPIO.setmode(GPIO.BCM)
SENSORS = [(ULTRASONIC_TRIG_FRONT, ULTRASONIC_ECHO_FRONT),
           (ULTRASONIC_TRIG_LEFT, ULTRASONIC_ECHO_LEFT),
           (ULTRASONIC_TRIG_RIGHT, ULTRASONIC_ECHO_RIGHT),
           (ULTRASONIC_TRIG_BACK, ULTRASONIC_ECHO_BACK),
           (ULTRASONIC_TRIG_TOP, ULTRASONIC_ECHO_TOP),
           (ULTRASONIC_TRIG_BOTTOM, ULTRASONIC_ECHO_BOTTOM)]

for trig, echo in SENSORS:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)

# Constants
EARTH_RADIUS = 6371000  # Radius of Earth in meters
OBSTACLE_THRESHOLD = 50  # cm
WAYPOINT_THRESHOLD = 2.0  # meters
FAILSAFE_TIMEOUT = 10  # seconds before failsafe engages
LANDING_HEIGHT = 0.38  # cm (ground detection threshold)
SMOOTH_TAKEOFF_HEIGHT = 2.0  # meters
SMOOTH_LANDING_HEIGHT = 0.5  # meters
PID_KP = 0.1  # Proportional gain
PID_KI = 0.01  # Integral gain
PID_KD = 0.05  # Derivative gain

# Function to calculate distance between two GPS points
def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c

# Function to get accurate ultrasonic distance
def get_distance(trig, echo, retries=3):
    for _ in range(retries):
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)
        start_time = time.time()
        stop_time = time.time()
        
        while GPIO.input(echo) == 0:
            start_time = time.time()
        
        while GPIO.input(echo) == 1:
            stop_time = time.time()
        
        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2  # Distance in cm
        
        if 2 < distance < 400:  # Valid range check
            return distance
    return 400  # Default to max distance if unreliable

# PID Control for stability
def pid_control(target_value, current_value, previous_error, integral, dt):
    error = target_value - current_value
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = PID_KP * error + PID_KI * integral + PID_KD * derivative
    return output, error, integral

class DroneNavigator(Node):
    def __init__(self):
        super().__init__('gps_ultrasonic_navigation')
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/mavros/imu/data", self.imu_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.current_gps = None
        self.current_imu = None
        self.waypoints = [(26.24870476474309, 78.1906974600621), (26.24870476474309, 78.1906974600621)]  # Example GPS waypoints
        self.home_gps = None
        self.last_heartbeat = time.time()
        self.current_waypoint_index = 0
        self.previous_error = 0
        self.integral = 0

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        self.last_heartbeat = time.time()  # Reset failsafe timer
        if not self.home_gps:
            self.home_gps = self.current_gps  # Set home location on first fix
        self.get_logger().info(f"Current GPS: {self.current_gps}")

    def imu_callback(self, msg):
        self.current_imu = msg
        self.get_logger().info(f"IMU Data: {self.current_imu}")

    def check_failsafe(self):
        if time.time() - self.last_heartbeat > FAILSAFE_TIMEOUT:
            self.get_logger().warn("Failsafe triggered! Returning to launch pad.")
            self.return_to_home()

    def return_to_home(self):
        if not self.home_gps:
            self.get_logger().warn("Home location not set. Stopping drone.")
            self.stop_drone()
            return
        
        self.get_logger().info("Navigating back to launch pad.")
        while haversine_distance(self.current_gps[0], self.current_gps[1], self.home_gps[0], self.home_gps[1]) > WAYPOINT_THRESHOLD:
            self.move_forward()
        
        self.stop_drone()
        self.get_logger().info("Drone safely landed at launch pad.")

    def stop_drone(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def smooth_takeoff(self):
        twist = Twist()
        twist.linear.z = 0.2  # Slowly rise
        self.cmd_pub.publish(twist)
        time.sleep(5)  # Gradually ascend for 5 seconds
        self.get_logger().info("Takeoff completed.")

    def navigate(self):
        rate = self.create_rate(10)
        while rclpy.ok() and self.current_waypoint_index < len(self.waypoints):
            self.check_failsafe()

            if self.current_gps:
                waypoint = self.waypoints[self.current_waypoint_index]
                distance_to_waypoint = haversine_distance(self.current_gps[0], self.current_gps[1], waypoint[0], waypoint[1])

                if distance_to_waypoint < WAYPOINT_THRESHOLD:
                    self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}")
                    self.current_waypoint_index += 1
                    continue

                distances = {sensor: get_distance(*sensor) for sensor in SENSORS}
                self.get_logger().info(f"Distances: {distances}")

                if all(dist > OBSTACLE_THRESHOLD for dist in distances.values()):
                    self.move_forward()
                else:
                    self.get_logger().warn("No clear path detected. Initiating landing.")
                    self.land_safely(distances)
            rate.sleep()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def land_safely(self, distances):
        twist = Twist()
        twist.linear.x = 0.0  # Stop moving forward
        twist.linear.z = -0.3  # Start descending
        self.cmd_pub.publish(twist)

        while distances[(ULTRASONIC_TRIG_BOTTOM, ULTRASONIC_ECHO_BOTTOM)] > LANDING_HEIGHT:
            distances[(ULTRASONIC_TRIG_BOTTOM, ULTRASONIC_ECHO_BOTTOM)] = get_distance(ULTRASONIC_TRIG_BOTTOM, ULTRASONIC_ECHO_BOTTOM)
            self.get_logger().info(f"Descending... Current height: {distances[(ULTRASONIC_TRIG_BOTTOM, ULTRASONIC_ECHO_BOTTOM)]} cm")
            
            # If the distance detected is close to LANDING_HEIGHT, we assume ground detection
            if distances[(ULTRASONIC_TRIG_BOTTOM, ULTRASONIC_ECHO_BOTTOM)] <= LANDING_HEIGHT:
                self.get_logger().info("Ground detected. Preparing for safe landing.")
                self.smooth_land()
                break
            time.sleep(0.5)

    def smooth_land(self):
        # After detecting ground, gently land by decreasing downward velocity
        twist = Twist()
        twist.linear.z = -0.1  # Continue gentle descent to land
        self.cmd_pub.publish(twist)
        time.sleep(2)  # Small delay to ensure landing
        self.stop_drone()
        self.get_logger().info("Successfully landed.")

    def avoid_obstacle(self, distances):
        twist = Twist()
        twist.linear.x = 0.0  # Stop

        if distances[(ULTRASONIC_TRIG_LEFT, ULTRASONIC_ECHO_LEFT)] > OBSTACLE_THRESHOLD:
            self.get_logger().info("Turning left to avoid obstacle.")
            twist.angular.z = 0.5
        elif distances[(ULTRASONIC_TRIG_RIGHT, ULTRASONIC_ECHO_RIGHT)] > OBSTACLE_THRESHOLD:
            self.get_logger().info("Turning right to avoid obstacle.")
            twist.angular.z = -0.5
        else:
            self.get_logger().warn("No clear path, stopping!")
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)
        time.sleep(2)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = DroneNavigator()
    navigator.smooth_takeoff()
    navigator.navigate()
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
    GPIO.cleanup()
