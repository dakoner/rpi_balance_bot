import sys
import math
from PyQt5 import QtCore
from mqtt_qobject import MqttClient
from mpu import mpu
from dual_tb9051ftg_rpi import motors, MAX_SPEED
from read_encoder import decoder
from simple_pid import PID
import pigpio
import time

TIMER_TICK = 10

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


        self.mpu = mpu()
        self.setupPid()
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
                self.mpu.filter()
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
            angle = self.mpu.angle()
            self.mot_left = self.pid_left(angle)
            self.mot_right = self.pid_right(angle)
            angle_error = self.angle_setpoint - angle

            motors.setSpeeds(-int(self.mot_left), -int(self.mot_right))
            self.last_ec_left = self.ec_left.pos
            self.last_ec_right = self.ec_right.pos
            msg = f'{{ "delta_left": {self.delta_left}, "delta_right": {self.delta_right}, "setpoint_left": {self.pid_left.setpoint}, "setpoint_right": {self.pid_right.setpoint}, "mot_left": {self.mot_left:.2f}, "mot_right": {self.mot_right:.2f}, "ec_left_pos": {self.ec_left.pos}, "ec_right_pos": {self.ec_right.pos}, "angle": {angle:.2f}, "angle_error": {angle_error:.2f}, "loop_time": {self.loop_time:.4f} }}'
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
