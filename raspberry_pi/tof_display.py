#!/usr/bin/env python3
"""
ToF Sensor Display - Shows 8x8 ToF depth data with plasma colormap.
Run this separately to visualize ToF readings in a window.
"""

import time
import numpy as np
import cv2
from tof_reader import ToFReader

# Settings
COLOR_MAP = cv2.COLORMAP_PLASMA  # Plasma colormap (purple -> yellow)
INVERSE = True                    # Close = bright, far = dark
MAX_DISTANCE = 5000               # Max distance in mm (5m)
WINDOW_SIZE = 480                 # Display window size in pixels
UPDATE_RATE = 0.05                # 20 Hz update rate


def main():
    """Main display loop."""
    print("Starting ToF Display...")
    
    try:
        tof = ToFReader()
    except Exception as e:
        print(f"Failed to initialize ToF sensor: {e}")
        return
    
    print("ToF Display ready. Press 'q' to quit.")
    
    while True:
        # Get ToF data (8x8 grid)
        data = tof.get_data()
        arr = np.array(data, dtype=np.float32)
        
        # Scale to 0-255 range
        arr = np.clip(arr, 0, MAX_DISTANCE)
        arr = arr * (255.0 / MAX_DISTANCE)
        
        # Inverse: close = bright (255), far = dark (0)
        if INVERSE:
            arr = 255.0 - arr
        
        # Convert to uint8
        arr = arr.astype(np.uint8)
        
        # Apply colormap
        colored = cv2.applyColorMap(arr, COLOR_MAP)
        
        # Resize to window size (nearest neighbor for blocky pixels)
        display = cv2.resize(colored, (WINDOW_SIZE, WINDOW_SIZE), 
                            interpolation=cv2.INTER_NEAREST)
        
        # Add grid lines
        grid_step = WINDOW_SIZE // 8
        for i in range(1, 8):
            pos = i * grid_step
            cv2.line(display, (pos, 0), (pos, WINDOW_SIZE), (50, 50, 50), 1)
            cv2.line(display, (0, pos), (WINDOW_SIZE, pos), (50, 50, 50), 1)
        
        # Add distance text for close objects
        for row in range(8):
            for col in range(8):
                dist = data[row][col]
                if dist > 0 and dist < 500:  # Show text for < 50cm
                    x = col * grid_step + grid_step // 2 - 15
                    y = row * grid_step + grid_step // 2 + 5
                    text = f"{int(dist/10)}"
                    cv2.putText(display, text, (x, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show display
        cv2.imshow('ToF Depth Sensor (Plasma)', display)
        
        # Handle key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
        time.sleep(UPDATE_RATE)
    
    # Cleanup
    tof.cleanup()
    cv2.destroyAllWindows()
    print("ToF Display closed.")


if __name__ == "__main__":
    main()

