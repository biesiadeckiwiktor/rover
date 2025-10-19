#!/usr/bin/env python3

import time
import threading
from gpiozero import DigitalInputDevice

class EncoderReader:
    def __init__(self):
        # GPIO pins for all 6 encoders (A/B swapped for inverted encoders)
        self.pins = {
            'FL_A': DigitalInputDevice(6, pull_up=True),   # Front Left A (swapped)
            'FL_B': DigitalInputDevice(5, pull_up=True),   # Front Left B (swapped)
            'FR_A': DigitalInputDevice(12, pull_up=True),  # Front Right A
            'FR_B': DigitalInputDevice(13, pull_up=True),  # Front Right B
            'ML_A': DigitalInputDevice(27, pull_up=True),  # Middle Left A (swapped)
            'ML_B': DigitalInputDevice(17, pull_up=True),  # Middle Left B (swapped)
            'MR_A': DigitalInputDevice(23, pull_up=True),  # Middle Right A
            'MR_B': DigitalInputDevice(24, pull_up=True),  # Middle Right B
            'RL_A': DigitalInputDevice(16, pull_up=True),  # Rear Left A
            'RL_B': DigitalInputDevice(19, pull_up=True),  # Rear Left B
            'RR_A': DigitalInputDevice(20, pull_up=True),  # Rear Right A
            'RR_B': DigitalInputDevice(21, pull_up=True),  # Rear Right B
        }
        
        # Encoder counts for all 6 wheels
        self.counts = {'FL': 0, 'FR': 0, 'ML': 0, 'MR': 0, 'RL': 0, 'RR': 0}
        self.last_states = {'FL': [0, 0], 'FR': [0, 0], 'ML': [0, 0], 'MR': [0, 0], 'RL': [0, 0], 'RR': [0, 0]}
        
        # Initialize states for all encoders
        self.last_states['FL'][0] = self.pins['FL_A'].value
        self.last_states['FL'][1] = self.pins['FL_B'].value
        self.last_states['FR'][0] = self.pins['FR_A'].value
        self.last_states['FR'][1] = self.pins['FR_B'].value
        self.last_states['ML'][0] = self.pins['ML_A'].value
        self.last_states['ML'][1] = self.pins['ML_B'].value
        self.last_states['MR'][0] = self.pins['MR_A'].value
        self.last_states['MR'][1] = self.pins['MR_B'].value
        self.last_states['RL'][0] = self.pins['RL_A'].value
        self.last_states['RL'][1] = self.pins['RL_B'].value
        self.last_states['RR'][0] = self.pins['RR_A'].value
        self.last_states['RR'][1] = self.pins['RR_B'].value
        
        # Start reading thread
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("Encoder reader initialized")
    
    def _read_loop(self):
        """Continuously read encoder states"""
        while self.running:
            self._update_encoder('FL', 'FL_A', 'FL_B')
            self._update_encoder('FR', 'FR_A', 'FR_B')
            self._update_encoder('ML', 'ML_A', 'ML_B')
            self._update_encoder('MR', 'MR_A', 'MR_B')
            self._update_encoder('RL', 'RL_A', 'RL_B')
            self._update_encoder('RR', 'RR_A', 'RR_B')
            time.sleep(0.001)
    
    def _update_encoder(self, name, pin_a_name, pin_b_name):
        """Update encoder count based on pin states - standard quadrature decoding"""
        current_a = self.pins[pin_a_name].value
        current_b = self.pins[pin_b_name].value
        last_a = self.last_states[name][0]
        last_b = self.last_states[name][1]
        
        # Proper quadrature: count on A transitions, use B for direction
        if current_a != last_a:
            if current_a == 1:  # Rising edge of A
                if current_b == 0:
                    self.counts[name] += 1  # Forward
                else:
                    self.counts[name] -= 1  # Reverse
            else:  # Falling edge of A  
                if current_b == 1:
                    self.counts[name] += 1  # Forward
                else:
                    self.counts[name] -= 1  # Reverse
        
        # Update last states
        self.last_states[name][0] = current_a
        self.last_states[name][1] = current_b
    
    def get_counts(self):
        """Get current encoder counts"""
        # Ensure all keys exist with default values
        default_counts = {'FL': 0, 'FR': 0, 'ML': 0, 'MR': 0, 'RL': 0, 'RR': 0}
        for key in default_counts:
            if key not in self.counts:
                self.counts[key] = 0
        return self.counts.copy()
    
    def reset_counts(self):
        """Reset encoder counts to zero"""
        self.counts = {'FL': 0, 'FR': 0, 'ML': 0, 'MR': 0, 'RL': 0, 'RR': 0}
    
    def cleanup(self):
        """Clean up and stop thread"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        for pin in self.pins.values():
            pin.close()

# Test script
if __name__ == "__main__":
    try:
        encoder = EncoderReader()
        print("Reading encoders... Press Ctrl+C to exit")
        
        while True:
            counts = encoder.get_counts()
            print(f"FL={counts['FL']:>4}, FR={counts['FR']:>4}, ML={counts['ML']:>4}, MR={counts['MR']:>4}, RL={counts['RL']:>4}, RR={counts['RR']:>4}")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        encoder.cleanup()
