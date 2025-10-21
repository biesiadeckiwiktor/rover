#!/usr/bin/env python3

import time
import math


class BalancePID:
    """
    Simple PID that keeps left and right wheel speeds matched.
    Uses encoder counts to measure actual speeds.
    """
    
    def __init__(self, encoder_reader, kp=0.1, ki=0.0, kd=0.0):
        self.encoder_reader = encoder_reader
        
        # PID gains - Very conservative P-only
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # State tracking
        self.last_counts = None
        self.last_time = None
        self.integral = 0.0
        self.last_error = 0.0
        
        # Heavy filtering for noisy speed measurements
        self.left_speed_filtered = 0.0
        self.right_speed_filtered = 0.0
        self.filter_alpha = 0.15  # Much stronger filtering
        
        # Constants
        self.wheel_diameter = 84  # mm
        self.counts_per_rev = 2100
        
    def reset(self):
        """Reset PID state."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_counts = None
        self.last_time = None
        self.left_speed_filtered = 0.0
        self.right_speed_filtered = 0.0
    
    def get_side_speeds(self):
        """Get left and right average speeds in counts/sec with heavy filtering."""
        if not self.encoder_reader:
            return 0.0, 0.0
        
        current_counts = self.encoder_reader.get_counts()
        current_time = time.time()
        
        if self.last_counts is None:
            self.last_counts = current_counts
            self.last_time = current_time
            return 0.0, 0.0
        
        dt = current_time - self.last_time
        if dt <= 0:
            return self.left_speed_filtered, self.right_speed_filtered
        
        # Calculate raw counts per second for each side
        left_speed_raw = ((current_counts['FL'] - self.last_counts['FL']) +
                          (current_counts['ML'] - self.last_counts['ML']) +
                          (current_counts['RL'] - self.last_counts['RL'])) / (3.0 * dt)
        
        right_speed_raw = ((current_counts['FR'] - self.last_counts['FR']) +
                           (current_counts['MR'] - self.last_counts['MR']) +
                           (current_counts['RR'] - self.last_counts['RR'])) / (3.0 * dt)
        
        # Apply heavy low-pass filter to smooth oscillations
        self.left_speed_filtered = (self.filter_alpha * left_speed_raw + 
                                     (1 - self.filter_alpha) * self.left_speed_filtered)
        self.right_speed_filtered = (self.filter_alpha * right_speed_raw + 
                                      (1 - self.filter_alpha) * self.right_speed_filtered)
        
        self.last_counts = current_counts
        self.last_time = current_time
        
        return self.left_speed_filtered, self.right_speed_filtered
    
    def update(self, base_speed):
        """
        Update PID to balance left/right speeds for straight driving.
        
        Args:
            base_speed: Desired motor speed (-255 to 255)
            
        Returns:
            (left_speed, right_speed) adjusted motor speeds
        """
        # Measure actual speeds
        left_speed, right_speed = self.get_side_speeds()
        
        # Calculate imbalance: positive means right is faster (pulling right)
        imbalance = right_speed - left_speed
        
        # PID control on the imbalance
        self.integral += imbalance * 0.02  # dt = 0.02 sec
        self.integral = max(-50, min(50, self.integral))  # Anti-windup
        
        derivative = (imbalance - self.last_error) / 0.02
        self.last_error = imbalance
        
        # Calculate correction
        correction = (self.kp * imbalance + 
                     self.ki * self.integral + 
                     self.kd * derivative)
        
        # Limit correction magnitude to prevent overcorrection
        correction = max(-15, min(15, correction))  # Reduced limit
        
        # Apply correction: if right is faster, slow it down
        left_out = base_speed + correction
        right_out = base_speed - correction
        
        # Clamp to motor limits
        left_out = int(max(-255, min(255, left_out)))
        right_out = int(max(-255, min(255, right_out)))
        
        return left_out, right_out
