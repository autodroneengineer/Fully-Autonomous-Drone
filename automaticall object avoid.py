import math
import time

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

# Ultrasonic sensor pins
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

class DroneNavigator:
    def __init__(self):
        rospy.init_node('gps_ultrasonic_navigation', anonymous=True)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.current_gps = None
        self.waypoints = [(28.6139, 77.2090), (28.6145, 77.2100)]  # Example GPS waypoints
        self.home_gps = None
        self.last_heartbeat = time.time()
        self.current_waypoint_index = 0

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        self.last_heartbeat = time.time()  # Reset failsafe timer
        
        if not self.home_gps:
            self.home_gps = self.current_gps  # Set home location on first fix
        
        rospy.loginfo(f"Current GPS: {self.current_gps}")

    def check_failsafe(self):
        if time.time() - self.last_heartbeat > FAILSAFE_TIMEOUT:
            rospy.logwarn("Failsafe triggered! Returning to launch pad.")
            self.return_to_home()

    def return_to_home(self):
        if not self.home_gps:
            rospy.logwarn("Home location not set. Stopping drone.")
            self.stop_drone()
            return
        
        rospy.loginfo("Navigating back to launch pad.")
        while haversine_distance(self.current_gps[0], self.current_gps[1], self.home_gps[0], self.home_gps[1]) > WAYPOINT_THRESHOLD:
            self.move_forward()
        
        self.stop_drone()
        rospy.loginfo("Drone safely landed at launch pad.")

    def stop_drone(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def navigate(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints):
            self.check_failsafe()
            
            if self.current_gps:
                waypoint = self.waypoints[self.current_waypoint_index]
                distance_to_waypoint = haversine_distance(self.current_gps[0], self.current_gps[1], waypoint[0], waypoint[1])
                
                if distance_to_waypoint < WAYPOINT_THRESHOLD:
                    rospy.loginfo(f"Reached waypoint {self.current_waypoint_index + 1}")
                    self.current_waypoint_index += 1
                    continue
                
                distances = {sensor: get_distance(*sensor) for sensor in SENSORS}
                rospy.loginfo(f"Distances: {distances}")
                
                if distances[(ULTRASONIC_TRIG_FRONT, ULTRASONIC_ECHO_FRONT)] < OBSTACLE_THRESHOLD:
                    rospy.logwarn("Obstacle detected! Deciding alternative path.")
                    self.avoid_obstacle(distances)
                else:
                    self.move_forward()
            rate.sleep()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def avoid_obstacle(self, distances):
        twist = Twist()
        twist.linear.x = 0.0
        
        if distances[(ULTRASONIC_TRIG_LEFT, ULTRASONIC_ECHO_LEFT)] > OBSTACLE_THRESHOLD:
            rospy.loginfo("Turning left to avoid obstacle.")
            twist.angular.z = 0.5
        elif distances[(ULTRASONIC_TRIG_RIGHT, ULTRASONIC_ECHO_RIGHT)] > OBSTACLE_THRESHOLD:
            rospy.loginfo("Turning right to avoid obstacle.")
            twist.angular.z = -0.5
        else:
            rospy.logwarn("No clear path, stopping!")
            twist.linear.x = 0.0
        
        self.cmd_pub.publish(twist)
        time.sleep(2)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        navigator = DroneNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
