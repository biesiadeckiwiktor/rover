#!/usr/bin/env python3

import math
import time

class Odometry:
    def __init__(self, encoder_reader=None):
        self.wheel_diameter = 84  # mm
        self.encoder_reader = encoder_reader
        self.counts_per_revolution = 2100  # encoder clicks per wheel revolution
        

    
    def get_distance_travelled(self):
        if not self.encoder_reader:
            return 0
        
        current_counts = self.encoder_reader.get_counts()
        average_counts = (current_counts['ML'] + current_counts['MR']) / 2
        distance_travelled = average_counts / self.counts_per_revolution * self.wheel_diameter * math.pi / 1000
        return distance_travelled


