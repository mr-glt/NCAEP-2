import time
import csv
from tentacle_pi.TSL2561 import TSL2561
from Adafruit_BME280 import *
from ctypes import *

sensor = BME280(mode=BME280_OSAMPLE_8)

tsl = TSL2561(0x39,"/dev/i2c-1")
tsl.enable_autogain()
tsl.set_time(0x00)

path = "lib/liblsm9ds1cwrapper.so"
lib = cdll.LoadLibrary(path)

lib.lsm9ds1_create.argtypes = []
lib.lsm9ds1_create.restype = c_void_p

lib.lsm9ds1_begin.argtypes = [c_void_p]
lib.lsm9ds1_begin.restype = None

lib.lsm9ds1_calibrate.argtypes = [c_void_p]
lib.lsm9ds1_calibrate.restype = None

lib.lsm9ds1_gyroAvailable.argtypes = [c_void_p]
lib.lsm9ds1_gyroAvailable.restype = c_int
lib.lsm9ds1_accelAvailable.argtypes = [c_void_p]
lib.lsm9ds1_accelAvailable.restype = c_int
lib.lsm9ds1_magAvailable.argtypes = [c_void_p]
lib.lsm9ds1_magAvailable.restype = c_int

lib.lsm9ds1_readGyro.argtypes = [c_void_p]
lib.lsm9ds1_readGyro.restype = c_int
lib.lsm9ds1_readAccel.argtypes = [c_void_p]
lib.lsm9ds1_readAccel.restype = c_int
lib.lsm9ds1_readMag.argtypes = [c_void_p]
lib.lsm9ds1_readMag.restype = c_int

lib.lsm9ds1_getGyroX.argtypes = [c_void_p]
lib.lsm9ds1_getGyroX.restype = c_float
lib.lsm9ds1_getGyroY.argtypes = [c_void_p]
lib.lsm9ds1_getGyroY.restype = c_float
lib.lsm9ds1_getGyroZ.argtypes = [c_void_p]
lib.lsm9ds1_getGyroZ.restype = c_float

lib.lsm9ds1_getAccelX.argtypes = [c_void_p]
lib.lsm9ds1_getAccelX.restype = c_float
lib.lsm9ds1_getAccelY.argtypes = [c_void_p]
lib.lsm9ds1_getAccelY.restype = c_float
lib.lsm9ds1_getAccelZ.argtypes = [c_void_p]
lib.lsm9ds1_getAccelZ.restype = c_float

lib.lsm9ds1_getMagX.argtypes = [c_void_p]
lib.lsm9ds1_getMagX.restype = c_float
lib.lsm9ds1_getMagY.argtypes = [c_void_p]
lib.lsm9ds1_getMagY.restype = c_float
lib.lsm9ds1_getMagZ.argtypes = [c_void_p]
lib.lsm9ds1_getMagZ.restype = c_float

lib.lsm9ds1_calcGyro.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcGyro.restype = c_float
lib.lsm9ds1_calcAccel.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcAccel.restype = c_float
lib.lsm9ds1_calcMag.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcMag.restype = c_float

#This is bad....I'm sorry
def pres2alt(pressure):
    alt = 44331.5 - 4946.62 * (pressure*100) ** (0.190263)
    return alt

if __name__ == "__main__":
    imu = lib.lsm9ds1_create()
    lib.lsm9ds1_begin(imu)
    if lib.lsm9ds1_begin(imu) == 0:
        print("Failed to communicate with LSM9DS1.")
        quit()
    lib.lsm9ds1_calibrate(imu)

while True:
    while lib.lsm9ds1_gyroAvailable(imu) == 0:
        pass
    lib.lsm9ds1_readGyro(imu)
    while lib.lsm9ds1_accelAvailable(imu) == 0:
        pass
    lib.lsm9ds1_readAccel(imu)
    while lib.lsm9ds1_magAvailable(imu) == 0:
        pass
    lib.lsm9ds1_readMag(imu)

    gx = lib.lsm9ds1_getGyroX(imu)
    gy = lib.lsm9ds1_getGyroY(imu)
    gz = lib.lsm9ds1_getGyroZ(imu)

    ax = lib.lsm9ds1_getAccelX(imu)
    ay = lib.lsm9ds1_getAccelY(imu)
    az = lib.lsm9ds1_getAccelZ(imu)

    mx = lib.lsm9ds1_getMagX(imu)
    my = lib.lsm9ds1_getMagY(imu)
    mz = lib.lsm9ds1_getMagZ(imu)

    cgx = lib.lsm9ds1_calcGyro(imu, gx)
    cgy = lib.lsm9ds1_calcGyro(imu, gy)
    cgz = lib.lsm9ds1_calcGyro(imu, gz)

    cax = lib.lsm9ds1_calcAccel(imu, ax)
    cay = lib.lsm9ds1_calcAccel(imu, ay)
    caz = lib.lsm9ds1_calcAccel(imu, az)

    cmx = lib.lsm9ds1_calcMag(imu, mx)
    cmy = lib.lsm9ds1_calcMag(imu, my)
    cmz = lib.lsm9ds1_calcMag(imu, mz)

    degrees = sensor.read_temperature()
    pascals = sensor.read_pressure()
    hectopascals = pascals / 100

    humidity = sensor.read_humidity()

    print("Gyro: %f, %f, %f [deg/s]" % (cgx, cgy, cgz))
    print("Accel: %f, %f, %f [Gs]" % (cax, cay, caz))
    print("Mag: %f, %f, %f [gauss]" % (cmx, cmy, cmz))
    print 'Timestamp = {0:0.3f}'.format(sensor.t_fine)
    print 'Temp      = {0:0.3f} deg C'.format(degrees)
    print 'Pressure  = {0:0.2f} hPa'.format(hectopascals)
    print 'Humidity  = {0:0.2f} %'.format(humidity)
    print "lux %s" % tsl.lux()
    print "Computed Altitude %s" % pres2alt(hectopascals)
    print "________________________"
    with open('datamain.csv', 'a') as csvfile:
        fieldnames = ['timestamp', 'gX', 'gY', 'gZ', 'aX', 'aY', 'aZ', 'mX', 'mY', 'mZ', 'tempC', 'hPa', 'humidity', 'lux', 'alt']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        #writer.writeheader()
        writer.writerow({'timestamp': cgx, 'gX': cgx, 'gY': cgy, 'gZ': cgz, 'aX': cax, 'aY': cay, 'aZ': caz, 'mX': cmx, 'mY': cmy, 'mZ': cmz, 'tempC': degrees, 'hPa': hectopascals, 'humidity': humidity, 'lux': tsl.lux(), 'alt': pres2alt(hectopascals)})

    time.sleep(0.5)
