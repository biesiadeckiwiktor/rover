#!/usr/bin/env python3

import time
import threading
import numpy

class ToFReader:
    def __init__(self):
        from vl53l5cx_ctypes import VL53L5CX
        self.sensor = VL53L5CX()
        self.sensor.set_resolution(8 * 8)
        self.sensor.start_ranging()
        self.data = [[0]*8 for _ in range(8)]
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
    
    def _read_loop(self):
        while self.running:
            try:
                if self.sensor.data_ready():
                    result = self.sensor.get_data()
                    grid = numpy.fliplr(numpy.flipud(numpy.array(result.distance_mm).reshape((8, 8)))).tolist()
                    
                    with self.lock:
                        self.data = grid
                
                time.sleep(0.05)
            except:
                time.sleep(0.1)
    
    def get_data(self):
        with self.lock:
            return [row[:] for row in self.data]
    
    def cleanup(self):
        self.running = False
        self.sensor.stop_ranging()

