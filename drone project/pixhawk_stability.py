import time

from pymavlink import mavutil

# Connect to Pixhawk
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Wait for the heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to Pixhawk!")

# PID Controller for Stability
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# Function to get gyroscope data from Pixhawk
def get_gyro_data():
    msg = connection.recv_match(type='RAW_IMU', blocking=True)
    if msg:
        gyro_x = msg.xgyro / 1000.0  # Convert to deg/sec
        gyro_y = msg.ygyro / 1000.0
        gyro_z = msg.zgyro / 1000.0
        return gyro_x, gyro_y, gyro_z
    return 0, 0, 0  # Default if no data

# Function to send motor speed commands
def set_motor_speed(motor1, motor2, motor3, motor4):
    connection.mav.rc_channels_override_send(
        connection.target_system, connection.target_component,
        motor1, motor2, motor3, motor4, 0, 0, 0, 0
    )

# Main Stability Control Loop
def main():
    pid_x = PIDController(1.0, 0.1, 0.05)
    pid_y = PIDController(1.0, 0.1, 0.05)

    try:
        while True:
            gyro_x, gyro_y, gyro_z = get_gyro_data()

            # Stability Adjustments
            correction_x = pid_x.compute(-gyro_x)  # Negative feedback
            correction_y = pid_y.compute(-gyro_y)

            # Adjust motor speeds
            base_speed = 1500  # Neutral throttle (adjust as needed)
            set_motor_speed(
                base_speed + correction_x,  # Motor 1
                base_speed - correction_x,  # Motor 2
                base_speed + correction_y,  # Motor 3
                base_speed - correction_y   # Motor 4
            )

            print(f"GyroX: {gyro_x:.2f}, GyroY: {gyro_y:.2f}, CorrectionX: {correction_x:.2f}, CorrectionY: {correction_y:.2f}")
            time.sleep(0.05)

    except KeyboardInterrupt:
        set_motor_speed(1500, 1500, 1500, 1500)  # Reset motors to neutral
        print("Motors stopped. Exiting...")

if __name__ == "__main__":
    main()

