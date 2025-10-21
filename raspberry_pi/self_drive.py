import math
import time
import logging

logger = logging.getLogger("self_drive")

class SelfDrive:
    def __init__(self, encoder_reader=None, driving_system=None, send_function=None, imu_reader=None, use_pid=True):
        self.wheel_diameter = 84  # mm
        self.counts_per_revolution = 2100
        self.encoder_reader = encoder_reader
        self.driving_system = driving_system
        self.send = send_function
        self.imu_reader = imu_reader
        
        self.pid = None
        if use_pid and encoder_reader:
            try:
                from pid_controller import BalancePID
                self.pid = BalancePID(encoder_reader, kp=0.1, ki=0.0, kd=0.0)
                if imu_reader:
                    logger.info("PID controller enabled with IMU fusion")
                else:
                    logger.info("PID controller enabled (encoder-only)")
            except Exception as e:
                logger.warning(f"PID init failed: {e}")

    def straight(self, speed, distance, direction="forward"):
        start = self.encoder_reader.get_counts()
        start_mid = (start['ML'] + start['MR']) / 2.0
        target_counts = distance * 1000 / (self.wheel_diameter * math.pi) * self.counts_per_revolution
        
        if direction == "backward":
            speed = -abs(speed)
        else:
            speed = abs(speed)
        
        if self.pid:
            self.pid.reset()
            while True:
                left_out, right_out = self.pid.update(speed)
                self.send(f"MOTORS:{left_out},{right_out},{left_out},{right_out},{left_out},{right_out}")
                current = self.encoder_reader.get_counts()
                if (current['ML'] + current['MR']) / 2.0 - start_mid >= target_counts:
                    break
                time.sleep(0.02)
        else:
            self.send(f"MOTORS:{speed},{speed},{speed},{speed},{speed},{speed}")
            while True:
                time.sleep(0.01)
                current = self.encoder_reader.get_counts()
                if (current['ML'] + current['MR']) / 2.0 - start_mid >= target_counts:
                    break
        
        self.send("MOTORS:0,0,0,0,0,0")

    def rotate_in_place(self, direction, fraction):
        if self.imu_reader:
            return self.rotate_in_place_imu(direction, fraction)
        
        logger.info("IMU not available, using encoder-based rotation (less accurate)")
        
        servo_angles = self.driving_system.set_turn_in_place_angles()
        self.send(f"STEER:{self.driving_system.get_steering_command_string(servo_angles)}")
        time.sleep(0.5)

        target_counts = (1000.0 * fraction / (self.wheel_diameter * math.pi) * self.counts_per_revolution) * 1.115
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
    
    def rotate_in_place_imu(self, direction, fraction):
        if not self.imu_reader:
            logger.warning("IMU not available, falling back to encoder-based rotation")
            return self.rotate_in_place(direction, fraction)
        
        target_angle = 360.0 * fraction
        
        servo_angles = self.driving_system.set_turn_in_place_angles()
        self.send(f"STEER:{self.driving_system.get_steering_command_string(servo_angles)}")
        time.sleep(0.5)
        
        left_cmd = -150 if direction == "left" else 150
        right_cmd = 150 if direction == "left" else -150
        
        self.send(f"MOTORS:{self.driving_system.get_motor_command_string([left_cmd, right_cmd, left_cmd, right_cmd, left_cmd, right_cmd])}")
        
        angle_turned = 0.0
        last_time = time.time()
        sample_count = 0
        
        logger.info(f"IMU rotate in place: target={target_angle:.1f}°, direction={direction}")
        
        while abs(angle_turned) < target_angle:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt > 0:
                imu_data = self.imu_reader.get_data()
                gyro_z = imu_data['gyro']['z']
                
                delta_angle = gyro_z * dt
                if direction == "right":
                    delta_angle = -delta_angle
                
                angle_turned += abs(delta_angle)
                last_time = current_time
                
                sample_count += 1
                if sample_count % 50 == 0:
                    logger.info(f"Rotation progress: {angle_turned:.1f}° / {target_angle:.1f}° (gyro_z={gyro_z:.1f}°/s)")
            
            time.sleep(0.01)
        
        self.send("MOTORS:0,0,0,0,0,0")
        self.send(f"STEER:{self.driving_system.get_steering_command_string(self.driving_system.center_steering())}")
        
        logger.info(f"IMU rotation complete: {angle_turned:.1f}° / {target_angle:.1f}° target")
    
    def get_heading(self):
        if not self.imu_reader:
            logger.warning("IMU not available, cannot get heading")
            return None
        
        imu_data = self.imu_reader.get_data()
        mag_x = imu_data['mag']['x']
        mag_y = imu_data['mag']['y']
        
        heading = math.degrees(math.atan2(mag_y, mag_x))
        if heading < 0:
            heading += 360
        
        return heading
    
    def turn_to_heading(self, target_heading, speed=100, tolerance=5.0, timeout=30.0):
        if not self.imu_reader:
            logger.warning("IMU not available, cannot turn to heading")
            return False
        
        current_heading = self.get_heading()
        if current_heading is None:
            return False
        
        angle_diff = target_heading - current_heading
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        if abs(angle_diff) <= tolerance:
            logger.info(f"Already at target heading: {current_heading:.1f}° (target={target_heading:.1f}°)")
            return True
        
        direction = "left" if angle_diff > 0 else "right"
        angle_to_turn = abs(angle_diff)
        
        logger.info(f"Turn to heading: current={current_heading:.1f}°, target={target_heading:.1f}°, turn {direction} {angle_to_turn:.1f}°")
        
        servo_angles = self.driving_system.set_turn_in_place_angles()
        self.send(f"STEER:{self.driving_system.get_steering_command_string(servo_angles)}")
        time.sleep(0.5)
        
        left_cmd = -speed if direction == "left" else speed
        right_cmd = speed if direction == "left" else -speed
        self.send(f"MOTORS:{self.driving_system.get_motor_command_string([left_cmd, right_cmd, left_cmd, right_cmd, left_cmd, right_cmd])}")
        
        angle_turned = 0.0
        start_time = time.time()
        last_time = time.time()
        sample_count = 0
        
        while time.time() - start_time < timeout:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt > 0:
                current_heading = self.get_heading()
                
                heading_error = target_heading - current_heading
                while heading_error > 180:
                    heading_error -= 360
                while heading_error < -180:
                    heading_error += 360
                
                if abs(heading_error) <= tolerance:
                    logger.info(f"Target heading reached: {current_heading:.1f}° (target={target_heading:.1f}°)")
                    self.send("MOTORS:0,0,0,0,0,0")
                    self.send(f"STEER:{self.driving_system.get_steering_command_string(self.driving_system.center_steering())}")
                    return True
                
                imu_data = self.imu_reader.get_data()
                gyro_z = imu_data['gyro']['z']
                delta_angle = gyro_z * dt
                if direction == "right":
                    delta_angle = -delta_angle
                angle_turned += abs(delta_angle)
                
                last_time = current_time
                
                sample_count += 1
                if sample_count % 50 == 0:
                    logger.info(f"Heading: {current_heading:.1f}° → {target_heading:.1f}° (error={heading_error:.1f}°, turned={angle_turned:.1f}°)")
            
            time.sleep(0.01)
        
        logger.warning(f"Turn to heading timed out after {timeout}s")
        self.send("MOTORS:0,0,0,0,0,0")
        self.send(f"STEER:{self.driving_system.get_steering_command_string(self.driving_system.center_steering())}")
        return False

    def make_specific_turn(self, speed, radius, direction, distance):
        
        if self.imu_reader:
            return self.make_specific_turn_imu(speed, radius, direction, distance)
        
        logger.info("IMU not available, using encoder-based arc measurement (less accurate)")
        
        radius_mm = radius * 1000
        angle_offset = math.degrees(math.asin(self.driving_system.wheelbase / (2 * radius_mm)))
        steering_angle = 90 - angle_offset if direction == "left" else 90 + angle_offset
        
        servo_angles = self.driving_system.calculate_servo_angles(steering_angle, radius_mm)
        self.send(f"STEER:{self.driving_system.get_steering_command_string(servo_angles)}")
        time.sleep(0.5)
        
        motor_speeds = self.driving_system.calculate_differential_speeds(speed, steering_angle)
        
        start_counts = self.encoder_reader.get_counts()
        start_avg = (start_counts['ML'] + start_counts['MR']) / 2
        arc_length_meters = 2 * math.pi * radius * distance
        distance_in_counts = arc_length_meters * 1000 / (self.wheel_diameter * math.pi) * self.counts_per_revolution
        target_counts = start_avg + (distance_in_counts * 0.8)
        
        # Arc/turn driving uses open-loop (differential speeds already calculated)
        self.send(f"MOTORS:{self.driving_system.get_motor_command_string(motor_speeds)}")
        while (self.encoder_reader.get_counts()['ML'] + self.encoder_reader.get_counts()['MR']) / 2 < target_counts:
            time.sleep(0.01)
        
        self.send("MOTORS:0,0,0,0,0,0")
        self.send(f"STEER:{self.driving_system.get_steering_command_string(self.driving_system.center_steering())}")

    def make_specific_turn_imu(self, speed, radius, direction, distance):
        if not self.imu_reader:
            logger.warning("IMU not available, falling back to encoder-based turn")
            return self.make_specific_turn(speed, radius, direction, distance)
        
        target_angle = 360.0 * distance  
        
        radius_mm = radius * 1000
        angle_offset = math.degrees(math.asin(self.driving_system.wheelbase / (2 * radius_mm)))
        steering_angle = 90 - angle_offset if direction == "left" else 90 + angle_offset
        
        servo_angles = self.driving_system.calculate_servo_angles(steering_angle, radius_mm)
        self.send(f"STEER:{self.driving_system.get_steering_command_string(servo_angles)}")
        time.sleep(0.5)
        
        motor_speeds = self.driving_system.calculate_differential_speeds(speed, steering_angle)
        self.send(f"MOTORS:{self.driving_system.get_motor_command_string(motor_speeds)}")
        
        angle_turned = 0.0
        last_time = time.time()
        sample_count = 0
        
        logger.info(f"IMU turn: target={target_angle:.1f}°, direction={direction}")
        
        while abs(angle_turned) < target_angle:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt > 0:
                imu_data = self.imu_reader.get_data()
                gyro_z = imu_data['gyro']['z']
                
                delta_angle = gyro_z * dt
                if direction == "left":
                    delta_angle = -delta_angle
                
                angle_turned += abs(delta_angle)
                last_time = current_time
                
                sample_count += 1
                if sample_count % 50 == 0:
                    logger.info(f"Turn progress: {angle_turned:.1f}° / {target_angle:.1f}° (gyro_z={gyro_z:.1f}°/s)")
            
            time.sleep(0.01)
        
        self.send("MOTORS:0,0,0,0,0,0")
        self.send(f"STEER:{self.driving_system.get_steering_command_string(self.driving_system.center_steering())}")
        
        logger.info(f"IMU turn complete: {angle_turned:.1f}° / {target_angle:.1f}° target")
