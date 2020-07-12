import sys
import math
from PyQt5 import QtCore
from mqtt_qobject import MqttClient
from mpu import mpu
from dual_tb9051ftg_rpi import motors, MAX_SPEED
from encoders import Encoders
from simple_pid import PID
import time

TIMER_TICK = 10
class Tui(QtCore.QObject):

    def __init__(self, app):
        self.app = app
        super(Tui, self).__init__()

        self.loop_time = 0.
        
        self.mot_left = 0
        self.mot_right = 0

        self.encoders = Encoders()
        self.ec_left_last = 0
        self.ec_right_last = 0
        
        self.mpu = mpu()

        self.Kp = 5
        self.Ki = 0.2
        self.Kd = 0.1
        self.left_setpoint = 0
        self.right_setpoint = 0
        
        self.setupPid()
        self.setupMqtt()
        
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timer_tick)
        self.timer.start(TIMER_TICK)

    def setupPid(self):
        SAMPLE_TIME = TIMER_TICK / 1000.
        self.pid_left = PID(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, setpoint=self.left_setpoint, sample_time=SAMPLE_TIME, output_limits=(-MAX_SPEED,MAX_SPEED))
        self.pid_right = PID(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, setpoint=self.right_setpoint, sample_time=SAMPLE_TIME, output_limits=(-MAX_SPEED,MAX_SPEED))


        
    def setupMqtt(self):
        self.client = MqttClient(self)
        self.client.hostname = "raspy"
        self.client.connectToHost()
        self.client.stateChanged.connect(self.on_stateChanged)
        self.client.messageSignal.connect(self.on_messageSignal)

        
    def timer_tick(self):
        try:
            t0 = time.time()
            # self.mpu.attitudeEuler()
            try:
                self.mpu.filter()
            except OSError:
                self.stop()

            self.pid_left.setpoint = self.left_setpoint
            self.pid_right.setpoint = self.right_setpoint
            self.pid_left.Kp = self.Kp
            self.pid_right.Kp = self.Kp
            self.pid_left.Ki = self.Ki
            self.pid_right.Ki = self.Ki
            self.pid_left.Kd = self.Kd
            self.pid_right.Kd = self.Kd

            delta_left = self.encoders.ec_left.pos - self.ec_left_last
            delta_right = self.encoders.ec_right.pos - self.ec_right_last
            self.ec_left_last = self.encoders.ec_left.pos
            self.ec_right_last = self.encoders.ec_right.pos
            
            self.mot_left = self.pid_left(delta_left)
            self.mot_right = self.pid_right(delta_right)
            # self.mot_left = self.left_setpoint
            # self.mot_right = self.right_setpoint
            motors.setSpeeds(int(self.mot_left), int(self.mot_right))
            
            
            msg = f'{{ "ec_left_pos": {self.encoders.ec_left.pos}, "ec_right_pos": {self.encoders.ec_right.pos}, "left_setpoint": {self.left_setpoint}, "right_setpoint": {self.right_setpoint}, "delta_left": {delta_left:.2f}, "delta_right": {delta_right:.2f}, "mot_left": {self.mot_left:.2f}, "mot_right": {self.mot_right:.2f}, "loop_time": {self.loop_time:.4f} }}'
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
            self.client.subscribe("robitt/control/left_setpoint")
            self.client.subscribe("robitt/control/right_setpoint")
            self.client.subscribe("robitt/control/p")
            self.client.subscribe("robitt/control/i")
            self.client.subscribe("robitt/control/d")

    @QtCore.pyqtSlot(str, str)
    def on_messageSignal(self, topic, payload):
        try:
            if topic == 'robitt/control/left_setpoint':
                self.left_setpoint = float(payload)
            if topic == 'robitt/control/right_setpoint':
                self.right_setpoint = float(payload)
            elif topic == 'robitt/control/p':
                self.Kp = float(payload)
            elif topic == 'robitt/control/i':
                self.Ki = float(payload)
            elif topic == 'robitt/control/d':
                self.Kd = float(payload)
        except ValueError as e:
            print(f"Failed to set {topic} to {payload}: {e}")

if __name__ == "__main__":
    app = QtCore.QCoreApplication(sys.argv)
    tui = Tui(app)
    sys.exit(app.exec_())
