from flask import Flask, render_template, request, jsonify, Response
import serial
import time
import threading
import cv2

from driving import DrivingSystem
from odometry import Odometry
from self_drive import SelfDrive

app = Flask(__name__)

try:
    from encoder_reader import EncoderReader
    encoder_reader = EncoderReader()
    print("Encoder reader: ON")
except Exception as e:
    print(f"Encoder reader: OFF ({e})")
    encoder_reader = None

try:
    from imu_reader import IMUReader
    imu_reader = IMUReader()
    print("IMU reader: ON")
except Exception as e:
    print(f"IMU reader: OFF ({e})")
    imu_reader = None

try:
    from tof_reader import ToFReader
    tof_reader = ToFReader()
    print("ToF reader: ON")
except Exception as e:
    print(f"ToF reader: OFF ({e})")
    tof_reader = None

# Initialize modular systems
arduino = None
driving_system = None
odometry = None
self_drive = None
camera = None
for idx in (0, 1, 2):
    cam = cv2.VideoCapture(idx)
    if cam.isOpened():
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera = cam
        print(f"Camera opened at index {idx}")
        break
if camera is None:
    print("No camera available; /video_feed will return 503")

# Speed control variables
current_steering_angle = 90
current_motor_speed = 0

try:
    arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2)
    print("Arduino connection established")
except Exception as e:
    print(f"Arduino connection failed: {e}")
    arduino = None

try:
    driving_system = DrivingSystem()
    print("Driving system initialized")
except Exception as e:
    print(f"Driving system error: {e}")
    driving_system = None

try:
    odometry = Odometry(encoder_reader)
    print("Odometry system initialized")
except Exception as e:
    print(f"Odometry system error: {e}")
    odometry = None

def send(cmd):
    if arduino:
        try:
            arduino.write(f"{cmd}\n".encode())
        except:
            pass

 

try:
    self_drive = SelfDrive(
        encoder_reader=encoder_reader,
        driving_system=driving_system,
        send_function=send,
        imu_reader=imu_reader,  # Enable IMU for accurate arc measurement
        use_pid=True
    )
    print("Self-drive system initialized")
except Exception as e:
    import traceback
    print(f"Self-drive system error: {e}")
    traceback.print_exc()
    self_drive = None

def continuous_sensor_display():
    """Display encoder and IMU data together"""
    import logging
    logger = logging.getLogger('sensors')
    
    while True:
        try:
            # Encoder data
            if encoder_reader:
                counts = encoder_reader.get_counts()
                logger.info(f"Encoders: FL={counts['FL']:>4} FR={counts['FR']:>4} ML={counts['ML']:>4} MR={counts['MR']:>4} RL={counts['RL']:>4} RR={counts['RR']:>4}")
            
            # Distance data
            if odometry and encoder_reader:
                distance = odometry.get_distance_travelled()
                logger.info(f"Distance travelled: {distance:.2f} m")
                time.sleep(1)
            
            # IMU data
            if imu_reader:
                imu = imu_reader.get_data()
                logger.info(f"Accel (g):  X={imu['accel']['x']:>7.3f}  Y={imu['accel']['y']:>7.3f}  Z={imu['accel']['z']:>7.3f}")
                logger.info(f"Gyro (°/s): X={imu['gyro']['x']:>7.1f}  Y={imu['gyro']['y']:>7.1f}  Z={imu['gyro']['z']:>7.1f}")
                logger.info(f"Mag (µT):   X={imu['mag']['x']:>7.1f}  Y={imu['mag']['y']:>7.1f}  Z={imu['mag']['z']:>7.1f}")
                logger.info(f"Temp: {imu['temp']:.1f}°C")
                time.sleep(1)
            
        except Exception as e:
            logger.error(f"Sensor display error: {e}")
            time.sleep(1)


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            success, frame = camera.read()
            if not success:
                time.sleep(0.05)
                continue
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/motor', methods=['POST'])
def motor():
    global current_motor_speed
    if not driving_system:
        return jsonify({'success': False, 'error': 'Driving system not available'})

    speed = request.json.get('speed', 0)
    current_motor_speed = speed

    speeds = driving_system.calculate_differential_speeds(abs(speed), current_steering_angle)

    if speed < 0:
        speeds = [-s for s in speeds]

    speed_string = driving_system.get_motor_command_string(speeds)
    send(f"MOTORS:{speed_string}")

    return jsonify({'success': True})

@app.route('/api/steering', methods=['POST'])
def steering():
    global current_steering_angle
    if not driving_system:
        return jsonify({'success': False, 'error': 'Driving system not available'})

    angle = request.json.get('angle', 90)
    current_steering_angle = driving_system.validate_steering_angle(angle)

    servo_angles = driving_system.calculate_servo_angles(current_steering_angle)
    servo_string = driving_system.get_steering_command_string(servo_angles)
    send(f"STEER:{servo_string}")

    return jsonify({'success': True})

@app.route('/api/camera/pan', methods=['POST'])
def pan():
    send(f"PAN:{request.json.get('angle', 90)}")
    return jsonify({'success': True})

@app.route('/api/camera/tilt', methods=['POST'])
def tilt():
    send(f"TILT:{request.json.get('angle', 90)}")
    return jsonify({'success': True})

@app.route('/api/stop', methods=['POST'])
def stop():
    send("STOP")
    return jsonify({'success': True})

@app.route('/api/center', methods=['POST'])
def center():
    global current_steering_angle
    if not driving_system:
        return jsonify({'success': False, 'error': 'Driving system not available'})

    current_steering_angle = 90

    servo_angles = driving_system.center_steering()
    servo_string = driving_system.get_steering_command_string(servo_angles)
    send(f"STEER:{servo_string}")

    if current_motor_speed != 0:
        speeds = driving_system.calculate_differential_speeds(abs(current_motor_speed), 90)

        if current_motor_speed < 0:
            speeds = [-s for s in speeds]

        speed_string = driving_system.get_motor_command_string(speeds)
        send(f"MOTORS:{speed_string}")

    return jsonify({'success': True})

@app.route('/api/set_turn_in_place', methods=['POST'])
def set_turn_in_place():
    """Set wheels to 45-degree angles for turn-in-place mode"""
    global current_steering_angle
    if not driving_system:
        return jsonify({'success': False, 'error': 'Driving system not available'})

    servo_angles = driving_system.set_turn_in_place_angles()
    servo_string = driving_system.get_steering_command_string(servo_angles)
    send(f"STEER:{servo_string}")

    return jsonify({'success': True})

@app.route('/api/turn_in_place', methods=['POST'])
def turn_in_place():
    if not driving_system:
        return jsonify({'success': False})

    direction = request.json.get('direction', 'forward')
    speed = request.json.get('speed', 0)

    if direction == 'forward':
        speeds = [speed, -speed, speed, -speed, speed, -speed]  
        #           FL,    FR,     ML,     MR,    RL,    RR
    else:
        speeds = [-speed, speed, -speed, speed, -speed, speed]  
        #            FL,   FR,      ML,    MR,     RL,    RR

    speed_string = driving_system.get_motor_command_string(speeds)
    send(f"MOTORS:{speed_string}")

    return jsonify({'success': True})



def cleanup():
    if arduino:
        arduino.close()
    if encoder_reader:
        encoder_reader.cleanup()
    if imu_reader:
        imu_reader.cleanup()
    if tof_reader:
        tof_reader.cleanup()
    global camera
    if camera is not None:
        camera.release()
        print("USB camera released")

if __name__ == '__main__':
    import logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('rover2.log')
        ]
    )
    
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    # ============================================================
   
    def planned_path():
        time.sleep(1)  
        
        if not self_drive:
            print("ERROR: self_drive not initialized - check encoder_reader and driving_system")
            return
            
        #self_drive.make_specific_turn(100, 1.0, "left", 0.25)
        self_drive.straight(150, 1.0, "forward")
self_drive.make_specific_turn(150, 0.50, "left", 0.5)
        #self_drive.straight(200, 1.0, "forward")
        self_drive.make_specific_turn(100, 1.0, "left", 0.25
    # ============================================================
    
    try:
        print(f"Rover ready - Encoder: {'ON' if encoder_reader else 'OFF'}, IMU: {'ON' if imu_reader else 'OFF'}")
        
        if encoder_reader or imu_reader:
            sensor_thread = threading.Thread(target=continuous_sensor_display, daemon=True)
            sensor_thread.start()
        
        path_thread = threading.Thread(target=planned_path, daemon=True)
        path_thread.start()
            
        app.run(host='0.0.0.0', port=5000)
    finally:
        cleanup() 


