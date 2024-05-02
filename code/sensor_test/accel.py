import smbus
import time

# MC3479 device address
MC3479_ADDR = 0x4C  

# MC3479 register addresses
REGISTER_ADDR = {
    'ACCEL_X_L': 0x0D,
    'ACCEL_X_H': 0x0E,
    'ACCEL_Y_L': 0x0F,
    'ACCEL_Y_H': 0x10,
    'ACCEL_Z_L': 0x11,
    'ACCEL_Z_H': 0x12,
}

def read_register(bus, register):
    return bus.read_byte_data(MC3479_ADDR, register)

def write_register(bus, register, value):
    bus.write_byte_data(MC3479_ADDR, register, value)

def read_accel_data(bus):
    # Read high and low bytes of X, Y, Z data
    accel_x = (read_register(bus, REGISTER_ADDR['ACCEL_X_H']) << 8) | read_register(bus, REGISTER_ADDR['ACCEL_X_L'])
    accel_y = (read_register(bus, REGISTER_ADDR['ACCEL_Y_H']) << 8) | read_register(bus, REGISTER_ADDR['ACCEL_Y_L'])
    accel_z = (read_register(bus, REGISTER_ADDR['ACCEL_Z_H']) << 8) | read_register(bus, REGISTER_ADDR['ACCEL_Z_L'])

    # Convert to signed 16-bit values
    if accel_x > 32767: accel_x -= 65536
    if accel_y > 32767: accel_y -= 65536
    if accel_z > 32767: accel_z -= 65536

    return accel_x, accel_y, accel_z

def main():
    bus = smbus.SMBus(1)
    write_register(bus, 0x07, 0x01)

    while True:
        # Read the accelerometer data
        accel_x, accel_y, accel_z = read_accel_data(bus)
        print(f'ACCEL_X: {accel_x}, ACCEL_Y: {accel_y}, ACCEL_Z: {accel_z}')

        time.sleep(0.1)

    bus.close()

if __name__ == '__main__':
    main()


