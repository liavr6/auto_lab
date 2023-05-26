import serial
import time
import math

# Serial connection
ser = serial.Serial('/dev/ttyACM0')
time.sleep(10)

# PID constants for position control
kp_pos = 0.5
ki_pos = 0.2
kd_pos = 0.1

# PID constants for heading control
kp_heading = 0.5
ki_heading = 0.2
kd_heading = 0.1

# Target position and initial variables
target_pos = [500, 0]  # Target position [x, y]
prev_error_pos = [0, 0]
integral_pos = [0, 0]

# Target heading and initial variables
target_heading = 0  # Target heading in degrees
prev_error_heading = 0
integral_heading = 0

# Initialize serial connection
def initialize_serial():
    ser.baudrate = 9600
    ser.timeout = 1
    time.sleep(2)

# Move the robot using motor commands
def move_robot(left_speed, right_speed):
    msg = str(left_speed) + ',' + str(right_speed) + '\r\n'
    ser.write(msg.encode('ascii'))

# Read current position from sensors or encoders
def read_current_position():
    # Replace with the code to read the current position from your sensors or encoders
    current_pos = [0, 0]  # [x, y]
    return current_pos

# Read current heading from sensors or gyro
def read_current_heading():
    # Replace with the code to read the current heading from your sensors or gyro
    current_heading = 0  # Heading in degrees
    return current_heading

# Perform PID control to reach the target position
def perform_position_control(target_pos):
    global prev_error_pos, integral_pos
    
    while True:
        # Read current position
        current_pos = read_current_position()
        
        # Calculate error
        error_pos = [target_pos[i] - current_pos[i] for i in range(2)]
        
        # Calculate PID terms
        proportional_pos = [kp_pos * error_pos[i] for i in range(2)]
        integral_pos = [integral_pos[i] + ki_pos * error_pos[i] for i in range(2)]
        derivative_pos = [(error_pos[i] - prev_error_pos[i]) * kd_pos for i in range(2)]
        
        # Calculate control output
        control_output_pos = [proportional_pos[i] + integral_pos[i] + derivative_pos[i] for i in range(2)]
        
        # Map control output to motor speeds
        left_speed = int(control_output_pos[0])
        right_speed = int(control_output_pos[1])
        
        # Move the robot
        move_robot(left_speed, right_speed)
        
        # Print debug information
        print("Target Position:", target_pos)
        print("Current Position:", current_pos)
        print("Control Output (Position):", control_output_pos)
        print("Left Speed:", left_speed)
        print("Right Speed:", right_speed)
        
        # Update previous error for next iteration
        prev_error_pos = error_pos
        
        # Delay between control loops
        time.sleep(0.1)

# Perform PID control to reach the target heading
def perform_heading_control(target_heading):
    global prev_error_heading, integral_heading
    
    while True:
        # Read current heading
        current_heading = read_current_heading()
        
        # Calculate error
        error_heading = target_heading - current_heading
        
        # Normalize error to be within -180 and 180 degrees
        if error_heading > 180:
            error_heading -= 360
        elif error_heading < -180:
            error_heading += 360
        
        # Calculate PID terms
        proportional_heading = kp_heading * error_heading
        integral_heading += ki_heading * error_heading
        derivative_heading = kd_heading * (error_heading - prev_error_heading)
        
        # Calculate control output
        control_output_heading = proportional_heading + integral_heading + derivative_heading
        
        # Map control output to rotation speed
        rotation_speed = int(control_output_heading)
        
        # Move the robot
        move_robot(rotation_speed, rotation_speed)
        
        # Print debug information
        print("Target Heading:", target_heading)
        print("Current Heading:", current_heading)
        print("Control Output (Heading):", control_output_heading)
        print("Rotation Speed:", rotation_speed)
        
        # Update previous error for next iteration
        prev_error_heading = error_heading
        
        # Delay between control loops
        time.sleep(0.1)

# Initialize the serial connection
initialize_serial()

# Perform PID control to reach the target position
perform_position_control(target_pos)

# Perform PID control to reach the target heading
perform_heading_control(target_heading)
