"""QMA6100P 3-axis accelerometer driver for ZX908 (I2C0, addr 0x12).

Hardware is QMA6100P-compatible (QST), NOT MC3416 as originally assumed.
Chip marking "7A58 RP2", I2C addr 0x12, Chip ID 0x90.
Class name kept as MC3416 for backward compatibility with tracker/mqtt.

Calibration: offset + 3x3 affine matrix (handles zero-g bias,
per-axis sensitivity, and cross-axis coupling).
3-position calibration required: flat, side1, side2.
"""

from machine import I2C
import utime
import math
import ujson

try:
    from modules.logging import getLogger
except ImportError:
    from usr.modules.logging import getLogger

log = getLogger("accel")

# I2C address
QMA6100P_ADDR = 0x12

# Registers
REG_CHIP_ID = 0x00       # Chip ID (0x90 for QMA6100P)
REG_XOUT_LSB = 0x01      # X-axis data low byte
REG_STATUS = 0x09         # Data ready status
REG_FSR_BW = 0x0F         # Full scale range [3:2] + bandwidth [7:4]
REG_PM = 0x11             # Power mode (bit7: 0=standby, 0x80=active)

# Range values (bits [3:2] of reg 0x0F)
RANGE_2G = 0x00
RANGE_4G = 0x04
RANGE_8G = 0x08
RANGE_16G = 0x0C

# Scale factors (LSB to mg) for 14-bit output — empirically calibrated
SCALE = {
    RANGE_2G: 0.136,
    RANGE_4G: 0.544,
    RANGE_8G: 1.097,
    RANGE_16G: 0.136,   # 16G wraps to 2G on this chip
}

CAL_FILE = '/usr/accel_cal.json'


class MC3416:
    """QMA6100P accelerometer driver (MC3416-compatible API)."""

    def __init__(self, bus=I2C.I2C0):
        self._i2c = I2C(bus, I2C.STANDARD_MODE)
        self._range = RANGE_8G
        self._scale = SCALE[RANGE_8G]
        self._movement_threshold = 80  # mg
        self._last_accel = (0, 0, 0)
        self._initialized = False
        # Calibration: cal = M * (raw - offset)
        self._offset = [0.0, 0.0, 0.0]
        self._mat = [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]

    def init(self):
        """Initialize sensor. Returns True if QMA6100P found."""
        chip_id = self._read_reg(REG_CHIP_ID)
        if chip_id != 0x90:
            log.error("QMA6100P not found (ID=0x%02X)" % chip_id)
            return False

        log.info("QMA6100P found (ID=0x%02X)" % chip_id)

        self._write_reg(REG_FSR_BW, self._range)
        utime.sleep_ms(5)

        self._write_reg(REG_PM, 0x80)
        utime.sleep_ms(20)

        pm = self._read_reg(REG_PM)
        if pm & 0x80 == 0:
            log.error("QMA6100P failed to activate (PM=0x%02X)" % pm)
            return False

        fsr = self._read_reg(REG_FSR_BW)
        self._range = fsr & 0x0C
        self._scale = SCALE.get(self._range, SCALE[RANGE_8G])
        log.info("Range: 0x%02X, scale: %.3f mg/LSB" % (self._range, self._scale))

        self._initialized = True
        self._load_calibration()
        return True

    def read_accel_raw(self):
        """Read raw acceleration in mg (no calibration). Returns (x, y, z)."""
        if not self._initialized:
            return (0, 0, 0)

        data = self._read_regs(REG_XOUT_LSB, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]

        # 14-bit signed: upper 14 bits, lower 2 are flags
        x = x >> 2
        y = y >> 2
        z = z >> 2
        if x > 8191:
            x -= 16384
        if y > 8191:
            y -= 16384
        if z > 8191:
            z -= 16384

        return (x * self._scale, y * self._scale, z * self._scale)

    def read_accel(self):
        """Read calibrated acceleration in mg. Returns (x, y, z).
        Note: sensor X/Y axes are swapped vs device convention."""
        rx, ry, rz = self.read_accel_raw()
        # Subtract offset
        dx = rx - self._offset[0]
        dy = ry - self._offset[1]
        dz = rz - self._offset[2]
        # Apply calibration matrix
        M = self._mat
        xm = M[0] * dx + M[1] * dy + M[2] * dz
        ym = M[3] * dx + M[4] * dy + M[5] * dz
        zm = M[6] * dx + M[7] * dy + M[8] * dz
        # Swap X/Y and negate X to match device physical axes
        self._last_accel = (-ym, xm, zm)
        return (-ym, xm, zm)

    def cal_measure(self, samples=50):
        """Measure average raw for calibration. Returns (x, y, z) in mg."""
        if not self._initialized:
            return None
        sx, sy, sz = 0.0, 0.0, 0.0
        for i in range(samples):
            x, y, z = self.read_accel_raw()
            sx += x
            sy += y
            sz += z
            utime.sleep_ms(20)
        return (sx / samples, sy / samples, sz / samples)

    def cal_compute(self, flat, side_x, side_y):
        """Compute calibration from 3 position measurements.
        flat: (x,y,z) raw mg when flat (Z up) -> true (0,0,1000)
        side_x: (x,y,z) raw mg on side (X axis gets gravity) -> true (±1000,0,0)
        side_y: (x,y,z) raw mg on side (Y axis gets gravity) -> true (0,±1000,0)
        Signs auto-detected from largest axis change vs flat."""
        # Auto-detect gravity direction for each side position
        dx_side = side_x[0] - flat[0]
        true_x = -1000.0 if dx_side < 0 else 1000.0

        dy_side = side_y[1] - flat[1]
        true_y = -1000.0 if dy_side < 0 else 1000.0

        # Offset: average of readings where that axis should be ~0
        ox = (flat[0] + side_y[0]) / 2.0   # X~0 in flat and side_y
        oy = (flat[1] + side_x[1]) / 2.0   # Y~0 in flat and side_x
        oz = (side_x[2] + side_y[2]) / 2.0  # Z~0 in both side positions

        self._offset = [ox, oy, oz]

        # Remove offset
        r1 = [flat[0] - ox, flat[1] - oy, flat[2] - oz]
        r2 = [side_x[0] - ox, side_x[1] - oy, side_x[2] - oz]
        r3 = [side_y[0] - ox, side_y[1] - oy, side_y[2] - oz]

        # Solve: TRUE = M * RAW  ->  M = TRUE * RAW^(-1)
        # RAW columns = r1, r2, r3;  TRUE columns = t1, t2, t3
        # t1=(0,0,1000), t2=(true_x,0,0), t3=(0,true_y,0)
        #
        # 3x3 matrix inversion inline (no numpy on MicroPython)
        a = r1[0]; b = r2[0]; c = r3[0]
        d = r1[1]; e = r2[1]; f = r3[1]
        g = r1[2]; h = r2[2]; k = r3[2]

        det = a*(e*k - f*h) - b*(d*k - f*g) + c*(d*h - e*g)
        if abs(det) < 0.001:
            log.error("Calibration matrix singular!")
            return None

        inv_det = 1.0 / det
        # Inverse of RAW matrix (cofactor transpose / det)
        ri = [
            (e*k - f*h) * inv_det, (c*h - b*k) * inv_det, (b*f - c*e) * inv_det,
            (f*g - d*k) * inv_det, (a*k - c*g) * inv_det, (c*d - a*f) * inv_det,
            (d*h - e*g) * inv_det, (b*g - a*h) * inv_det, (a*e - b*d) * inv_det,
        ]

        # M = TRUE * RAW_inv
        # TRUE = [[0, true_x, 0], [0, 0, true_y], [1000, 0, 0]]
        # M[row][col] = sum_j TRUE[row][j] * RAW_inv[j][col]
        self._mat = [
            true_x * ri[3], true_x * ri[4], true_x * ri[5],
            true_y * ri[6], true_y * ri[7], true_y * ri[8],
            1000.0 * ri[0], 1000.0 * ri[1], 1000.0 * ri[2],
        ]

        self._save_calibration()
        log.info("Cal offset: (%.1f, %.1f, %.1f)" % (ox, oy, oz))
        log.info("Cal matrix diag: (%.3f, %.3f, %.3f)" % (self._mat[0], self._mat[4], self._mat[8]))

        # Verify
        x, y, z = self.read_accel()
        mag = math.sqrt(x*x + y*y + z*z)
        log.info("Verify: (%.0f, %.0f, %.0f) |g|=%.0f" % (x, y, z, mag))
        return (x, y, z)

    def detect_movement(self):
        """Detect movement based on acceleration change. Returns True if moving."""
        if not self._initialized:
            return False
        x, y, z = self.read_accel()
        total = math.sqrt(x * x + y * y + z * z)
        deviation = abs(total - 1000)
        return deviation > self._movement_threshold

    def get_last_accel(self):
        """Return last read acceleration (x, y, z) in mg."""
        return self._last_accel

    def standby(self):
        """Put sensor in standby (low power)."""
        if self._initialized:
            self._write_reg(REG_PM, 0x00)

    def wake(self):
        """Wake sensor from standby."""
        if self._initialized:
            self._write_reg(REG_PM, 0x80)
            utime.sleep_ms(20)

    def _load_calibration(self):
        try:
            f = open(CAL_FILE, 'r')
            cal = ujson.load(f)
            f.close()
            self._offset = cal['offset']
            self._mat = cal['mat']
            log.info("Loaded calibration (offset+matrix)")
        except:
            log.info("No calibration, using defaults")

    def _save_calibration(self):
        try:
            f = open(CAL_FILE, 'w')
            ujson.dump({'offset': self._offset, 'mat': self._mat}, f)
            f.close()
            log.info("Calibration saved")
        except Exception as e:
            log.error("Cal save failed: %s" % e)

    def _read_reg(self, reg):
        r = bytearray([reg])
        d = bytearray(1)
        self._i2c.read(QMA6100P_ADDR, r, 1, d, 1, 0)
        return d[0]

    def _read_regs(self, reg, n):
        r = bytearray([reg])
        d = bytearray(n)
        self._i2c.read(QMA6100P_ADDR, r, 1, d, n, 0)
        return d

    def _write_reg(self, reg, val):
        r = bytearray([reg])
        d = bytearray([val])
        self._i2c.write(QMA6100P_ADDR, r, 1, d, 1)
