from mpu9250_complementary import MPU
# from mpu9250_madgwick import MPU

GYRO = 2000      # 250, 500, 1000, 2000 [deg/s]
ACC = 16         # 2, 4, 7, 16 [g]
MAG = 16        # 14, 16 [bit]
TAU = 0.98

class mpu:
    def __init__(self):
        self.mpu = MPU(GYRO, ACC, TAU)

        # Set up the IMU and mag sensors
        self.mpu.setUp()
        
        # self.mpu.setUpIMU()
        # self.mpu.setUpMAG()

        # Calibrate the mag or provide values that have been verified with the visualizer
        # mpu.calibrateMagGuide()
        # bias = [145, 145, -155]
        # scale = [1.10, 1.05, 1.05]
        # self.mpu.setMagCalibration(bias, scale)
        # Calibrate gyro with N points
        self.mpu.calibrateGyro(500)

    def timer(self):
        self.mpu.attitudeEuler()

    def filter(self):
        # self.mpu.processValues()
        # self.mpu.madgwickFilter(self.mpu.ax, -self.mpu.ay, self.mpu.az,
        #                         math.radians(self.mpu.gx), -math.radians(self.mpu.gy), -math.radians(self.mpu.gz), 
        #                         self.mpu.my, -self.mpu.mx, self.mpu.mz, 100)
        
        # self.mpu.attitudeEuler()
        self.mpu.compFilter()

    def angle(self):
        return self.mpu.roll
