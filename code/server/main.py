import io
import logging
import socketserver
from http import server
from threading import Condition
import json
import smbus
import time
import RPi.GPIO as GPIO

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

from ds18b20 import DS18B20 #https://github.com/rgbkrk/ds18b20

# MC3479 ACCEL address
MC3479_ADDR = 0x4C  

# IAM-20380 GYRO address
IAM_20380_ADDR = 0x69 

# Garmin LIDAR address
LIDAR_ADDR = 0x62

# registers for I2C devices
REGISTER_ADDR = {
    # MC3479 ACCEL register addresses
    'ACCEL_X_L': 0x0D,
    'ACCEL_X_H': 0x0E,
    'ACCEL_Y_L': 0x0F,
    'ACCEL_Y_H': 0x10,
    'ACCEL_Z_L': 0x11,
    'ACCEL_Z_H': 0x12,
    # IAM-20380 GYRO register addresses
    'GYRO_X_H': 0x43,
    'GYRO_X_L': 0x44,
    'GYRO_Y_H': 0x45,
    'GYRO_Y_L': 0x46,
    'GYRO_Z_H': 0x47,
    'GYRO_Z_L': 0x48,
    # GARMIN LIDAR
    'LIDAR_CTRL' : 0x00,
    'LIDAR_STAT' : 0x01,
    'LIDAR_DIST_H' : 0x0F,
    'LIDAR_DIST_L' : 0x10,
}

# ONE WIRE TEMP SENSORS
TEMP_SENSOR = {
    "EXT_TEMP": "0b2324dea7d4",
    "INT_TEMP": "0b2344cf6e44",
}

# read a register from a device
def read_register(addr, bus, register):
    return bus.read_byte_data(addr, register)

# write to the register of a device
def write_register(addr, bus, register, value):
    bus.write_byte_data(addr, register, value)

# read accelerometer data and return 
def read_accel_data(bus):
    try:
        accel_x = (read_register(MC3479_ADDR, bus, REGISTER_ADDR['ACCEL_X_H']) << 8) | read_register(MC3479_ADDR, bus, REGISTER_ADDR['ACCEL_X_L'])
        accel_y = (read_register(MC3479_ADDR, bus, REGISTER_ADDR['ACCEL_Y_H']) << 8) | read_register(MC3479_ADDR, bus, REGISTER_ADDR['ACCEL_Y_L'])
        accel_z = (read_register(MC3479_ADDR, bus, REGISTER_ADDR['ACCEL_Z_H']) << 8) | read_register(MC3479_ADDR, bus, REGISTER_ADDR['ACCEL_Z_L'])

    # convert to signed 16-bit values
        if accel_x > 32767: accel_x -= 65536
        if accel_y > 32767: accel_y -= 65536
        if accel_z > 32767: accel_z -= 65536

        return accel_x, accel_y, accel_z
    except:
        print("SENSOR ERROR: ACCEL NOT FOUND")
        return -1

# read gyro and return xyz data
def read_gyro_data(bus): 
    try:
        gyro_x = (read_register(IAM_20380_ADDR, bus, REGISTER_ADDR['GYRO_X_H'])  << 8) | read_register(IAM_20380_ADDR, bus, REGISTER_ADDR['GYRO_X_L'])
        gyro_y = (read_register(IAM_20380_ADDR, bus, REGISTER_ADDR['GYRO_Y_H'])  << 8) | read_register(IAM_20380_ADDR, bus, REGISTER_ADDR['GYRO_Y_L'])
        gyro_z = (read_register(IAM_20380_ADDR, bus, REGISTER_ADDR['GYRO_Z_H'])  << 8) | read_register(IAM_20380_ADDR, bus, REGISTER_ADDR['GYRO_Z_L'])

        # convert to signed 16-bit values
        if gyro_x > 32767: gyro_x -= 65536
        if gyro_y > 32767: gyro_y -= 65536
        if gyro_z > 32767: gyro_z -= 65536

        return gyro_x, gyro_y, gyro_z
    except:
        print("SENSOR ERROR: GRYO NOT FOUND")
        return -1

# read lidar distance and return in cm
def read_lidar_distance(bus):
    try:
        # initiate measurement
        write_register(LIDAR_ADDR, bus, REGISTER_ADDR['LIDAR_CTRL'], 0x04)
        i = 0
        # wait for the measurement to be taken
        while (not (read_register(LIDAR_ADDR, bus, REGISTER_ADDR['LIDAR_STAT']) & 0x01)) and (i < 200000):
            i += 1

        if i == 200000: return -1

        # Read two bytes from the distance register
        distance = (read_register(LIDAR_ADDR, bus, REGISTER_ADDR['LIDAR_DIST_H']) << 8) | read_register(LIDAR_ADDR, bus, REGISTER_ADDR['LIDAR_DIST_L'])

        return distance
    except:
        print("SENSOR ERROR: LIDAR NOT FOUND")
        return -1
        
# read one-wire temperature sensor and return value in deg F
def read_temp_sensor(address):
    try:
        sensor = DS18B20(address)
        return round(sensor.get_temperature(DS18B20.DEGREES_F), 2)
    except:
        print("SENSOR ERROR: TEMPERATURE NOT FOUND")
        return -1

class MotorController:
    def __init__(self):
        print("Init motors\n")
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(14, GPIO.OUT)
        GPIO.setup(15, GPIO.OUT)
        GPIO.setup(16, GPIO.OUT)
        GPIO.setup(17, GPIO.OUT)

        self.motors = {
            'front_left': GPIO.PWM(12, 100),
            'front_right': GPIO.PWM(13, 100),
            'center_left': GPIO.PWM(15, 100),
            'center_right': GPIO.PWM(14, 100),
            'rear_left': GPIO.PWM(17, 100),
            'rear_right': GPIO.PWM(16, 100),
        }

        for motor in self.motors.values():
            motor.start(0)

            
            #motor.ChangeDutyCycle(15)

    def move_up(self):
        print("Move up\n")
        self.motors['front_left'].ChangeDutyCycle(16)
        self.motors['front_right'].ChangeDutyCycle(16)
        self.motors['center_left'].ChangeDutyCycle(15)
        self.motors['center_right'].ChangeDutyCycle(15)
        self.motors['rear_left'].ChangeDutyCycle(16)
        self.motors['rear_right'].ChangeDutyCycle(16)
        
    def move_down(self):
        print("Move down\n")
        self.motors['front_left'].ChangeDutyCycle(14)
        self.motors['front_right'].ChangeDutyCycle(14)
        self.motors['center_left'].ChangeDutyCycle(15)
        self.motors['center_right'].ChangeDutyCycle(15)
        self.motors['rear_left'].ChangeDutyCycle(14)
        self.motors['rear_right'].ChangeDutyCycle(14)


    def move_left(self):
        print("Move left\n")
        self.motors['front_left'].ChangeDutyCycle(15)
        self.motors['front_right'].ChangeDutyCycle(15)
        self.motors['center_left'].ChangeDutyCycle(16)
        self.motors['center_right'].ChangeDutyCycle(14)
        self.motors['rear_left'].ChangeDutyCycle(15)
        self.motors['rear_right'].ChangeDutyCycle(15)
        

    def move_right(self):
        print("Move right\n")
        self.motors['front_left'].ChangeDutyCycle(15)
        self.motors['front_right'].ChangeDutyCycle(15)
        self.motors['center_left'].ChangeDutyCycle(14)
        self.motors['center_right'].ChangeDutyCycle(16)
        self.motors['rear_left'].ChangeDutyCycle(15)
        self.motors['rear_right'].ChangeDutyCycle(15)

    def move_forward(self):
        print("Move forward\n")
        self.motors['front_left'].ChangeDutyCycle(15)
        self.motors['front_right'].ChangeDutyCycle(15)
        self.motors['center_left'].ChangeDutyCycle(16)
        self.motors['center_right'].ChangeDutyCycle(16)
        self.motors['rear_left'].ChangeDutyCycle(15)
        self.motors['rear_right'].ChangeDutyCycle(15)

    def move_backward(self):
        print("Move backward\n")
        self.motors['front_left'].ChangeDutyCycle(15)
        self.motors['front_right'].ChangeDutyCycle(15)
        self.motors['center_left'].ChangeDutyCycle(14)
        self.motors['center_right'].ChangeDutyCycle(14)
        self.motors['rear_left'].ChangeDutyCycle(15)
        self.motors['rear_right'].ChangeDutyCycle(15)

    def stop(self):
        print("Stop\n")
        for motor in self.motors.values():
            motor.ChangeDutyCycle(15)

    def cleanup(self):
        print("Cleanup\n")
        for motor in self.motors.values():
            motor.stop()
        GPIO.cleanup()

# init i2c bus
bus = smbus.SMBus(1)


# init motor controller
rov_motors = MotorController()

# init accel
write_register(MC3479_ADDR, bus, 0x07, 0x01)

# Webpage
# This code was taken from the picamera2 library mjpg_server.py example
# It is intended to stream the Pi Camera to a simple locally hosted webpage
# Code modified to add ROV sensors and controls
# The base code for handling camera streaming and web server stuff isn't commented
# I'll try my best to explain our modifications via comments
html_path = "index.html"
PAGE = ""

with open(html_path, 'r') as f:
    PAGE = f.read()
    

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

# Streaming handler
class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/data.json': # here is were we stream ROV sensors
            accel_x, accel_y, accel_z = read_accel_data(bus) # read accel
            gyro_x, gyro_y, gyro_z = read_gyro_data(bus)     # read gyro
            lidar_distance = read_lidar_distance(bus)        # read LIDAR 
            internal_temp = read_temp_sensor(TEMP_SENSOR['INT_TEMP'])
            external_temp = read_temp_sensor(TEMP_SENSOR['EXT_TEMP'])
            data = {                                         # make a dict with sensor data
                'accel_x': accel_x,
                'accel_y': accel_y,
                'accel_z': accel_z,
                'gyro_x': gyro_x,
                'gyro_y': gyro_y,
                'gyro_z': gyro_z,
                'lidar_distance': lidar_distance,
                'internal_temp': internal_temp,
                'external_temp' : external_temp,
            }
            content = json.dumps(data).encode('utf-8')      # encode and save the data to the webserver
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()
    
    # handling POST requests from client 
    # buttons to move ROV
    def do_POST(self):
        if self.path.startswith('/move/'):
            direction = self.path.split('/')[2]
            if direction == 'up':
                self.move_up()
            elif direction == 'down':
                self.move_down()
            elif direction == 'left':
                self.move_left()
            elif direction == 'right':
                self.move_right()
            elif direction == 'forward':
                self.move_forward()
            elif direction == 'backward':
                self.move_backward()
            elif direction == 'stop':
                self.stop()
            self.send_response(200)
            self.end_headers()
        else:
            self.send_error(404)
            self.end_headers()

    # these just call the functions associated with the class MotorController. PWM handled there.
    def move_up(self):
        rov_motors.move_up()

    def move_down(self):
        rov_motors.move_down()
        
    def move_left(self):
        rov_motors.move_left()
    
    def move_right(self):
        rov_motors.move_right()
        
    def move_forward(self):
        rov_motors.move_forward()
        
    def move_backward(self):
        rov_motors.move_backward()

    def stop(self):
        rov_motors.stop()
        

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
output = StreamingOutput()
picam2.start_recording(JpegEncoder(), FileOutput(output))

try:
    address = ('', 8000)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()
finally:
    picam2.stop_recording()