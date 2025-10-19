#!/usr/bin/env python3

import time
import threading

class IMUReader:
    def __init__(self):
        from icm20948 import ICM20948
        self.imu = ICM20948()
        self.data = {'accel': {'x': 0, 'y': 0, 'z': 0}, 'gyro': {'x': 0, 'y': 0, 'z': 0}, 'mag': {'x': 0, 'y': 0, 'z': 0}, 'temp': 0}
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
    
    def _read_loop(self):
        while self.running:
            try:
                mx, my, mz = self.imu.read_magnetometer_data()
                ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
                temp = self.imu.read_temperature()
                with self.lock:
                    self.data = {
                        'accel': {'x': ax, 'y': ay, 'z': az},
                        'gyro': {'x': gx, 'y': gy, 'z': gz},
                        'mag': {'x': mx, 'y': my, 'z': mz},
                        'temp': temp,
                    }
                time.sleep(0.05)
            except:
                time.sleep(0.1)
    
    def get_data(self):
        with self.lock:
            return self.data.copy()
    
    def cleanup(self):
        self.running = False

