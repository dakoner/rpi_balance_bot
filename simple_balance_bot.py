import sys
import math
from PyQt5 import QtCore
from mqtt_qobject import MqttClient
from mpu import mpu
from dual_tb9051ftg_rpi import motors, MAX_SPEED
from simple_pid import PID
import pigpio
import time

TIMER_TICK = 10

class Tui(QtCore.QObject):

    def __init__(self, app):
        self.app = app
        super(Tui, self).__init__()

        self.loop_time = 0.
        
        self.Kp = 25
        self.Ki = 5.0
        self.Kd = 1.25
        self.angle_setpoint = 0

        self.mot = 0

        self.mpu = mpu()
        self.setupPid()
        self.setupMqtt()
        
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timer_tick)
        self.timer.start(TIMER_TICK)

    def setupPid(self):
        SAMPLE_TIME = TIMER_TICK / 1000.
        self.pid = PID(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, setpoint=self.angle_setpoint, sample_time=SAMPLE_TIME, output_limits=(-MAX_SPEED,MAX_SPEED))
        self.pi = pigpio.pi()


        
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

            self.pid.setpoint = self.angle_setpoint
            self.pid.Kp = self.Kp
            self.pid.Ki = self.Ki
            self.pid.Kd = self.Kd

            angle = self.mpu.angle()
            self.mot = self.pid(angle)
            angle_error = self.angle_setpoint - angle

            motors.setSpeeds(-int(self.mot), -int(self.mot))
            msg = f'{{ "angle_setpoint": {self.pid.setpoint}, "mot": {self.mot:.2f}, "angle": {angle:.2f}, "angle_error": {angle_error:.2f}, "loop_time": {self.loop_time:.4f} }}'
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
