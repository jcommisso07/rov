import smbus
import time

# IAM-20380 device address
IAM_20380_ADDR = 0x69

# IAM-20380 register addresses
REGISTER_ADDR = {
    'GYRO_X_H': 0x43,
    'GYRO_X_L': 0x44,
    'GYRO_Y_H': 0x45,
    'GYRO_Y_L': 0x46,
    'GYRO_Z_H': 0x47,
    'GYRO_Z_L': 0x48,
}

def read_register(bus, register):
    return bus.read_byte_data(IAM_20380_ADDR, register)

def read_gyro_data(bus):
    # Read high and low bytes of X, Y, Z data
    gyro_x = (read_register(bus, REGISTER_ADDR['GYRO_X_H']) << 8) | read_register(bus, REGISTER_ADDR['GYRO_X_L'])
    gyro_y = (read_register(bus, REGISTER_ADDR['GYRO_Y_H']) << 8) | read_register(bus, REGISTER_ADDR['GYRO_Y_L'])
    gyro_z = (read_register(bus, REGISTER_ADDR['GYRO_Z_H']) << 8) | read_register(bus, REGISTER_ADDR['GYRO_Z_L'])

    # Convert to signed 16-bit values
    if gyro_x > 32767: gyro_x -= 65536
    if gyro_y > 32767: gyro_y -= 65536
    if gyro_z > 32767: gyro_z -= 65536

    return gyro_x, gyro_y, gyro_z

def main():
    bus = smbus.SMBus(1)

    while True:
        # Read the gyroscope data
        gyro_x, gyro_y, gyro_z = read_gyro_data(bus)
        print(f'GYRO_X: {gyro_x}, GYRO_Y: {gyro_y}, GYRO_Z: {gyro_z}')

        time.sleep(1)

    bus.close()

if __name__ == '__main__':
    main()
