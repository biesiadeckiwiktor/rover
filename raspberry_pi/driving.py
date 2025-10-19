import math

class DrivingSystem:
    def __init__(self):
        self.track_width = 285
        self.middle_track_width = 310
        self.wheelbase = 275
        self.wheel_diameter = 84
        self.num_motors = 6
        self.front_left_center = 65
        self.front_right_center = 76
        self.rear_left_center = 85
        self.rear_right_center = 92
        self.front_left_dir = 1
        self.front_right_dir = 1

    def calculate_differential_speeds(self, base_speed, steering_angle):
        if steering_angle == 90 or abs(math.radians(steering_angle - 90)) < 0.01:
            return [base_speed] * self.num_motors

        steering_angle_rad = math.radians(steering_angle - 90)
        turn_radius = self.wheelbase / (2 * abs(math.sin(steering_angle_rad)))
        turn_center_x = turn_radius if steering_angle > 90 else -turn_radius

        wheel_positions = [
            (-self.track_width/2, self.wheelbase/2),
            (self.track_width/2, self.wheelbase/2),
            (-self.middle_track_width/2, 0),
            (self.middle_track_width/2, 0),
            (-self.track_width/2, -self.wheelbase/2),
            (self.track_width/2, -self.wheelbase/2)
        ]

        speeds = []
        for x, y in wheel_positions:
            distance_from_center = math.sqrt((x - turn_center_x)**2 + y**2)
            wheel_speed = max(0, min(255, int((distance_from_center / turn_radius) * base_speed)))
            speeds.append(wheel_speed)

        return speeds

    def calculate_servo_angles(self, steering_angle, turn_radius=None):
        if abs(steering_angle - 90) == 0:
            return [self.front_left_center, self.front_right_center, self.rear_left_center, self.rear_right_center]

        if turn_radius is None:
            steering_angle_rad = math.radians(steering_angle - 90)
            turn_radius = self.wheelbase / (2 * abs(math.sin(steering_angle_rad)))
        
        if steering_angle > 90:
            center_x = turn_radius
            fl_angle = math.degrees(math.atan2(self.wheelbase/2, center_x - (-self.track_width/2)))
            fr_angle = math.degrees(math.atan2(self.wheelbase/2, center_x - self.track_width/2))
            rl_angle = math.degrees(math.atan2(-self.wheelbase/2, center_x - (-self.track_width/2)))
            rr_angle = math.degrees(math.atan2(-self.wheelbase/2, center_x - self.track_width/2))
            
            angles = [
                self.front_left_center + fl_angle,
                self.front_right_center + fr_angle,
                self.rear_left_center - rl_angle,
                self.rear_right_center + rr_angle
            ]
        else:
            center_x = -turn_radius
            fl_angle = math.degrees(math.atan2(self.wheelbase/2, -self.track_width/2 - center_x))
            fr_angle = math.degrees(math.atan2(self.wheelbase/2, self.track_width/2 - center_x))
            rl_angle = math.degrees(math.atan2(-self.wheelbase/2, -self.track_width/2 - center_x))
            rr_angle = math.degrees(math.atan2(-self.wheelbase/2, self.track_width/2 - center_x))
            
            angles = [
                self.front_left_center - fl_angle,
                self.front_right_center - fr_angle,
                self.rear_left_center - rl_angle,
                self.rear_right_center - rr_angle
            ]

        if self.front_left_dir == -1:
            angles[0] = 180 - angles[0]
        if self.front_right_dir == -1:
            angles[1] = 180 - angles[1]

        return [max(0, min(180, angle)) for angle in angles]

    def validate_motor_speeds(self, speeds):
        if not isinstance(speeds, list) or len(speeds) != self.num_motors:
            return [0] * self.num_motors

        validated = []
        for speed in speeds:
            try:
                speed = int(speed)
                speed = max(-255, min(255, speed))
                validated.append(speed)
            except:
                validated.append(0)

        return validated

    def validate_steering_angle(self, angle):
        try:
            angle = float(angle)
            return max(0, min(180, angle))
        except:
            return 90

    def get_motor_command_string(self, speeds):
        validated_speeds = self.validate_motor_speeds(speeds)
        return ','.join(map(str, validated_speeds))

    def get_steering_command_string(self, angles):
        validated = []
        for angle in angles:
            try:
                validated.append(int(max(0, min(180, float(angle)))))
            except:
                validated.append(90)
        return ','.join(map(str, validated))

    def emergency_stop(self):
        return [0] * self.num_motors

    def center_steering(self):
        return [self.front_left_center, self.front_right_center, self.rear_left_center, self.rear_right_center]

    def set_turn_in_place_angles(self):
        return [self.front_left_center + 45, self.front_right_center - 45, 
                self.rear_left_center - 45, self.rear_right_center + 45]
