import math
import time
import logging
from pid_controller import PIController

logger = logging.getLogger("self_drive")

class SelfDrive:
    def __init__(self, encoder_reader=None, driving_system=None, send_function=None, imu_reader=None):
        self.wheel_diameter = 84
        self.encoder_reader = encoder_reader
        self.driving_system = driving_system
        self.send = send_function
        self.imu_reader = imu_reader
        self.rotate_full_turn_meters = 1.0

    def straight(self, speed, distance, direction):
        pid = PIController(0.5, 0.02)
        
        if direction == "forward":
            self.send(f"MOTORS:{speed},{speed},{speed},{speed},{speed},{speed}")
        else:
            self.send(f"MOTORS:-{speed},-{speed},-{speed},-{speed},-{speed},-{speed}")

        current_counts = self.encoder_reader.get_counts()
        average_of_all_counts = (current_counts['ML'] + current_counts['MR'] + current_counts['FL'] + current_counts['FR'] + current_counts['RL'] + current_counts['RR']) / 6
        start_left = (current_counts['FL'] + current_counts['ML'] + current_counts['RL']) / 3
        start_right = (current_counts['FR'] + current_counts['MR'] + current_counts['RR']) / 3
        distance_in_counts = distance * 1000 * 1000 / (self.wheel_diameter * math.pi)
        final_counts = average_of_all_counts + distance_in_counts
        
        while average_of_all_counts < final_counts:
            time.sleep(0.01)
            current_counts = self.encoder_reader.get_counts()
            average_of_all_counts = (current_counts['ML'] + current_counts['MR'] + current_counts['FL'] + current_counts['FR'] + current_counts['RL'] + current_counts['RR']) / 6
            
            left_avg = (current_counts['FL'] + current_counts['ML'] + current_counts['RL']) / 3
            right_avg = (current_counts['FR'] + current_counts['MR'] + current_counts['RR']) / 3
            error = (left_avg - start_left) - (right_avg - start_right)
            adjustment = pid.get_value(error)
            
            base_speed = speed if direction == "forward" else -speed
            left_speed = int(base_speed - adjustment if direction == "forward" else base_speed + adjustment)
            right_speed = int(base_speed + adjustment if direction == "forward" else base_speed - adjustment)
            self.send(f"MOTORS:{left_speed},{right_speed},{left_speed},{right_speed},{left_speed},{right_speed}")
        
        self.send("MOTORS:0,0,0,0,0,0")

    def rotate_in_place_encoder(self, direction, fraction):
        servo_angles = self.driving_system.set_turn_in_place_angles()
        self.send(f"STEER:{self.driving_system.get_steering_command_string(servo_angles)}")
        time.sleep(0.5)

        target_counts = (self.rotate_full_turn_meters * 1000.0 * fraction * 1000 / (self.wheel_diameter * math.pi)) * 1.115

        left_cmd = -100 if direction == "left" else 100
        right_cmd = 100 if direction == "left" else -100

        start = self.encoder_reader.get_counts()
        start_left = (start['FL'] + start['ML'] + start['RL']) / 3.0
        start_right = (start['FR'] + start['MR'] + start['RR']) / 3.0

        self.send(f"MOTORS:{self.driving_system.get_motor_command_string([left_cmd, right_cmd, left_cmd, right_cmd, left_cmd, right_cmd])}")

        while True:
            time.sleep(0.01)
            c = self.encoder_reader.get_counts()
            left_avg = (c['FL'] + c['ML'] + c['RL']) / 3.0
            right_avg = (c['FR'] + c['MR'] + c['RR']) / 3.0
            if max(abs(left_avg - start_left), abs(right_avg - start_right)) >= target_counts:
                break

        self.send("MOTORS:0,0,0,0,0,0")
        self.send(f"STEER:{self.driving_system.get_steering_command_string(self.driving_system.center_steering())}")

    def make_specific_turn(self, speed, radius, direction, distance):
        radius_mm = radius * 1000
        angle_offset = math.degrees(math.asin(self.driving_system.wheelbase / (2 * radius_mm)))
        steering_angle = 90 - angle_offset if direction == "left" else 90 + angle_offset
        
        servo_angles = self.driving_system.calculate_servo_angles(steering_angle, radius_mm)
        self.send(f"STEER:{self.driving_system.get_steering_command_string(servo_angles)}")
        time.sleep(0.5)
        
        motor_speeds = self.driving_system.calculate_differential_speeds(speed, steering_angle)
        self.send(f"MOTORS:{self.driving_system.get_motor_command_string(motor_speeds)}")
        
        start_counts = self.encoder_reader.get_counts()
        start_avg = (start_counts['ML'] + start_counts['MR']) / 2
        
        arc_length_meters = 2 * math.pi * radius * distance
        distance_in_counts = arc_length_meters * 1000 * 1000 / (self.wheel_diameter * math.pi)
        target_counts = start_avg + (distance_in_counts * 0.8)
        
        while (self.encoder_reader.get_counts()['ML'] + self.encoder_reader.get_counts()['MR']) / 2 < target_counts:
            time.sleep(0.01)
        
        self.send("MOTORS:0,0,0,0,0,0")
        self.send(f"STEER:{self.driving_system.get_steering_command_string(self.driving_system.center_steering())}")

    def make_specific_turn_imu(self, speed, radius, direction, distance):
        """Turn using IMU gyroscope for accurate angle measurement"""
        if not self.imu_reader:
            logger.warning("IMU not available, falling back to encoder-based turn")
            return self.make_specific_turn(speed, radius, direction, distance)
        
        # Calculate target angle from distance fraction
        target_angle = 360.0 * distance  # distance is fraction of circle (0.25 = 90°)
        
        radius_mm = radius * 1000
        angle_offset = math.degrees(math.asin(self.driving_system.wheelbase / (2 * radius_mm)))
        steering_angle = 90 - angle_offset if direction == "left" else 90 + angle_offset
        
        # Set steering
        servo_angles = self.driving_system.calculate_servo_angles(steering_angle, radius_mm)
        self.send(f"STEER:{self.driving_system.get_steering_command_string(servo_angles)}")
        time.sleep(0.5)
        
        # Start motors
        motor_speeds = self.driving_system.calculate_differential_speeds(speed, steering_angle)
        self.send(f"MOTORS:{self.driving_system.get_motor_command_string(motor_speeds)}")
        
        # Track angle using gyroscope integration
        angle_turned = 0.0
        last_time = time.time()
        
        logger.info(f"IMU turn: target={target_angle:.1f}°, direction={direction}")
        
        while abs(angle_turned) < target_angle:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt > 0:
                # Get gyroscope Z-axis (yaw rate)
                imu_data = self.imu_reader.get_data()
                gyro_z = imu_data['gyro']['z']
                
                # Integrate: angle = rate × time
                # Negate for left turns (gyro_z is positive for right rotation)
                delta_angle = gyro_z * dt
                if direction == "left":
                    delta_angle = -delta_angle
                
                angle_turned += abs(delta_angle)
                
                logger.debug(f"Turned: {angle_turned:.1f}° / {target_angle:.1f}° (gyro_z={gyro_z:.1f}°/s)")
                
                last_time = current_time
            
            time.sleep(0.01)
        
        # Stop
        self.send("MOTORS:0,0,0,0,0,0")
        self.send(f"STEER:{self.driving_system.get_steering_command_string(self.driving_system.center_steering())}")
        
        logger.info(f"IMU turn complete: {angle_turned:.1f}° turned")
