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

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ULTRASONIC_TRIG_FRONT, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO_FRONT, GPIO.IN)
GPIO.setup(ULTRASONIC_TRIG_LEFT, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO_LEFT, GPIO.IN)
GPIO.setup(ULTRASONIC_TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO_RIGHT, GPIO.IN)

# Constants
EARTH_RADIUS = 6371000  # Radius of Earth in meters
OBSTACLE_THRESHOLD = 50  # cm
WAYPOINT_THRESHOLD = 2.0  # meters

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
        self.current_waypoint_index = 0

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        rospy.loginfo(f"Current GPS: {self.current_gps}")

    def navigate(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints):
            if self.current_gps:
                waypoint = self.waypoints[self.current_waypoint_index]
                distance_to_waypoint = haversine_distance(self.current_gps[0], self.current_gps[1], waypoint[0], waypoint[1])
                
                if distance_to_waypoint < WAYPOINT_THRESHOLD:
                    rospy.loginfo(f"Reached waypoint {self.current_waypoint_index + 1}")
                    self.current_waypoint_index += 1
                    continue
                
                distance_front = get_distance(ULTRASONIC_TRIG_FRONT, ULTRASONIC_ECHO_FRONT)
                distance_left = get_distance(ULTRASONIC_TRIG_LEFT, ULTRASONIC_ECHO_LEFT)
                distance_right = get_distance(ULTRASONIC_TRIG_RIGHT, ULTRASONIC_ECHO_RIGHT)
                rospy.loginfo(f"Front: {distance_front}cm, Left: {distance_left}cm, Right: {distance_right}cm")
                
                if distance_front < OBSTACLE_THRESHOLD:
                    rospy.logwarn("Obstacle detected! Deciding alternative path.")
                    self.avoid_obstacle(distance_left, distance_right)
                else:
                    self.move_forward()
            rate.sleep()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 1.0  # Move forward
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def avoid_obstacle(self, distance_left, distance_right):
        twist = Twist()
        twist.linear.x = 0.0  # Stop
        
        if distance_left > OBSTACLE_THRESHOLD and distance_left > distance_right:
            rospy.loginfo("Turning left to avoid obstacle.")
            twist.angular.z = 0.5
        elif distance_right > OBSTACLE_THRESHOLD:
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
