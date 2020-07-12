import sys
import math
from PyQt5 import QtCore
from mqtt_qobject import MqttClient
from pid import pid
from dual_tb9051ftg_rpi import motors, MAX_SPEED
import time

TIMER_TICK = 10
class Tui(QtCore.QObject):

    def __init__(self, app):
        self.app = app
        super(Tui, self).__init__()

        self.loop_time = 0.
        self.enabled = 1
        
        self.pid = pid(TIMER_TICK)
        self.setupMqtt()
        
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timer_tick)
        self.timer.start(TIMER_TICK)

    def setupMqtt(self):
        self.client = MqttClient(self)
        self.client.hostname = "raspy"
        self.client.connectToHost()
        self.client.stateChanged.connect(self.on_stateChanged)
        self.client.messageSignal.connect(self.on_messageSignal)

        
    def timer_tick(self):
        try:
            t0 = time.time()
            if self.enabled:
                motors.enable()
            else:
                motors.disable()

            try:
                pid_msg = self.pid.update()
            except OSError:
                self.stop()
            
            msg = f'{{"pid": {{ {pid_msg} }}, "loop_time": {self.loop_time:.4f}, "enabled": {self.enabled} }}'
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
            self.client.subscribe("robitt/control/enabled")
            self.client.subscribe("robitt/control/left_setpoint")
            self.client.subscribe("robitt/control/right_setpoint")
            self.client.subscribe("robitt/control/p")
            self.client.subscribe("robitt/control/i")
            self.client.subscribe("robitt/control/d")

    @QtCore.pyqtSlot(str, str)
    def on_messageSignal(self, topic, payload):
        try:
            if topic == 'robitt/control/left_setpoint':
                self.pid.left_setpoint = float(payload)
            if topic == 'robitt/control/right_setpoint':
                self.pid.right_setpoint = float(payload)
            elif topic == 'robitt/control/p':
                self.pid.Kp = float(payload)
            elif topic == 'robitt/control/i':
                self.pid.Ki = float(payload)
            elif topic == 'robitt/control/d':
                self.pid.Kd = float(payload)
            elif topic == 'robitt/control/enabled':
                self.enabled = int(payload)
        except ValueError as e:
            print(f"Failed to set {topic} to {payload}: {e}")

if __name__ == "__main__":
    app = QtCore.QCoreApplication(sys.argv)
    tui = Tui(app)
    sys.exit(app.exec_())
