import sys
import math
from PyQt5 import QtCore
from mqtt_qobject import MqttClient
#from pololu_drv8835_rpi import motors, MAX_SPEED
from dual_tb9051ftg_rpi import motors, MAX_SPEED
from read_encoder import decoder
from mpu9250_complementary import MPU
# from mpu9250_madgwick import MPU
from simple_pid import PID
import pigpio
import time

TIMER_TICK = 10
GYRO = 2000      # 250, 500, 1000, 2000 [deg/s]
ACC = 16         # 2, 4, 7, 16 [g]
MAG = 16        # 14, 16 [bit]
TAU = 0.98

DECODER_LEFT_PINS = (20,21)
DECODER_RIGHT_PINS = (19,16)
class EncoderCallback:
    def __init__(self, name):
        self.name = name
        self.pos = 0
        self.t = time.time()
      
    def callback(self, way):
        self.pos += way

class Tui(QtCore.QObject):

    def __init__(self, app):
        self.app = app
        super(Tui, self).__init__()

        self.loop_time = 0.
        
        self.Kp = 25
        self.Ki = 5.0
        self.Kd = 1.25
        self.angle_setpoint = 0

        self.last_ec_left = 0
        self.last_ec_right = 0
        self.left = 0
        self.right = 0
        self.delta_left = 0
        self.delta_right = 0
        self.mot_left = 0
        self.mot_right = 0


        self.setupPid()
        self.setupGyro()
        self.setupMqtt()
        
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timer_tick)
        self.timer.start(TIMER_TICK)

    def setupPid(self):
        SAMPLE_TIME = TIMER_TICK / 1000.
        self.pid_left = PID(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, setpoint=self.angle_setpoint, sample_time=SAMPLE_TIME, output_limits=(-MAX_SPEED,MAX_SPEED))
        self.pid_right = PID(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, setpoint=self.angle_setpoint, sample_time=SAMPLE_TIME, output_limits=(-MAX_SPEED,MAX_SPEED))
        self.pi = pigpio.pi()
        self.ec_left = EncoderCallback("enc1")
        self.dec_left = decoder(self.pi, *DECODER_LEFT_PINS, self.ec_left.callback)
        self.ec_right = EncoderCallback("enc2")
        self.dec_right = decoder(self.pi, *DECODER_RIGHT_PINS, self.ec_right.callback)

    def setupGyro(self):
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

        
    def setupMqtt(self):
        self.client = MqttClient(self)
        self.client.hostname = "raspy"
        self.client.connectToHost()
        self.client.stateChanged.connect(self.on_stateChanged)
        self.client.messageSignal.connect(self.on_messageSignal)

        
    def timer_tick(self):
        try:
            t0 = time.time()
            # self.mpu.processValues()
            # self.mpu.madgwickFilter(self.mpu.ax, -self.mpu.ay, self.mpu.az,
            #                         math.radians(self.mpu.gx), -math.radians(self.mpu.gy), -math.radians(self.mpu.gz), 
            #                         self.mpu.my, -self.mpu.mx, self.mpu.mz, 100)

            # self.mpu.attitudeEuler()
            try:
                self.mpu.compFilter()
            except OSError:
                self.stop()

            self.pid_left.setpoint = self.angle_setpoint
            self.pid_right.setpoint = self.angle_setpoint
            self.pid_left.Kp = self.Kp
            self.pid_right.Kp = self.Kp
            self.pid_left.Ki = self.Ki
            self.pid_right.Ki = self.Ki
            self.pid_left.Kd = self.Kd
            self.pid_right.Kd = self.Kd

            self.delta_left = self.ec_left.pos - self.last_ec_left
            self.delta_right = self.ec_right.pos - self.last_ec_right
            self.mot_left = self.pid_left(self.mpu.roll)
            self.mot_right = self.pid_right(self.mpu.roll)
            self.angle_error = self.angle_setpoint - self.mpu.roll

            motors.setSpeeds(-int(self.mot_left), -int(self.mot_right))
            self.last_ec_left = self.ec_left.pos
            self.last_ec_right = self.ec_right.pos
            msg = f'{{ "delta_left": {self.delta_left}, "delta_right": {self.delta_right}, "setpoint_left": {self.pid_left.setpoint}, "setpoint_right": {self.pid_right.setpoint}, "mot_left": {self.mot_left:.2f}, "mot_right": {self.mot_right:.2f}, "ec_left_pos": {self.ec_left.pos}, "ec_right_pos": {self.ec_right.pos}, "pitch": {self.mpu.pitch:.2f}, "roll": {self.mpu.roll:.2f}, "yaw": {self.mpu.yaw:.2f}, "angle_error": {self.angle_error}, "loop_time": {self.loop_time} }}'
            self.client.publish("robitt/motor", msg)

            t1 = time.time()
            self.loop_time = t1-t0
        except KeyboardInterrupt:
            self.stop()


    def stop(self):
        motors.setSpeeds(0,0)
        self.app.quit()
        
        
    @QtCore.pyqtSlot(int)
    def on_stateChanged(self, state):
        if state == MqttClient.Connected:
            self.client.subscribe("robitt/control/angle_setpoint")
            self.client.subscribe("robitt/control/p")
            self.client.subscribe("robitt/control/i")
            self.client.subscribe("robitt/control/d")

    @QtCore.pyqtSlot(str, str)
    def on_messageSignal(self, topic, payload):
        if topic == 'robitt/control/angle_setpoint':
            self.angle_setpoint = float(payload)
        elif topic == 'robitt/control/p':
            self.Kp = float(payload)
        elif topic == 'robitt/control/i':
            self.Ki = float(payload)
        elif topic == 'robitt/control/d':
            self.Kd = float(payload)

if __name__ == "__main__":
    app = QtCore.QCoreApplication(sys.argv)
    tui = Tui(app)
    sys.exit(app.exec_())
