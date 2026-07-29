from machine import I2C
import utime

class MC3416:
    def __init__(self, bus=I2C.I2C0, addr=0x12):
        self.bus = bus
        self.addr = addr
        self.i2c = None

    def init(self):
        try:
            self.i2c = I2C(self.bus, I2C.STANDARD_MODE)
            # Read Chip ID (Register 0x00, expecting 0x90)
            r, d = bytearray([0x00]), bytearray(1)
            self.i2c.read(self.addr, r, 1, d, 1, 0)
            chip_id = d[0]
            if chip_id != 0x90:
                print('Unexpected Chip ID:', hex(chip_id))
                return False
            # Wake up sensor (Register 0x11 -> 0x80 active mode)
            self.i2c.write(self.addr, bytearray([0x11]), 1, bytearray([0x80]), 1)
            utime.sleep_ms(20)
            print('QMA6100P found! ID:', hex(chip_id))
            return True
        except Exception as e:
            print('Init error:', e)
            return False

    def read_accel(self):
        try:
            # Read 6 bytes starting at XOUT_LSB (0x01)
            r, d = bytearray([0x01]), bytearray(6)
            self.i2c.read(self.addr, r, 1, d, 6, 0)
            # Shift right 2 bits (14-bit resolution)
            x = ((d[1] << 8) | d[0]) >> 2
            y = ((d[3] << 8) | d[2]) >> 2
            z = ((d[5] << 8) | d[4]) >> 2
            if x > 8191: x -= 16384
            if y > 8191: y -= 16384
            if z > 8191: z -= 16384
            # Convert to mg (scale ~1.097 for 8g range)
            return (round(x * 1.097, 1), round(y * 1.097, 1), round(z * 1.097, 1))
        except Exception as e:
            print('Read error:', e)
            return (0.0, 0.0, 0.0)

sensor = MC3416()
if sensor.init():
    print('Real Sensor Reading (mg):', sensor.read_accel())
