from simple_pid import PID
from mpu import mpu
from dual_tb9051ftg_rpi import motors, MAX_SPEED

class pid:
    def __init__(self, timer_tick):

        sample_time = timer_tick / 1000.
        self.Kp = 20
        self.Ki = 5
        self.Kd = -0.2
        self.left_setpoint = 0

        self.pid = PID(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, setpoint=self.left_setpoint, sample_time=sample_time, output_limits=(-MAX_SPEED,MAX_SPEED))

        self.mpu = mpu()
        self.motors = motors

    def update(self):
        self.pid.setpoint = self.left_setpoint
        self.pid.Kp = self.Kp
        self.pid.Ki = self.Ki
        self.pid.Kd = self.Kd

        self.mpu.filter()
        angle = self.mpu.angle()
        self.mot = self.pid(angle)
        self.motors.setSpeeds(-int(self.mot), -int(self.mot))

        return f'"left_setpoint": {self.left_setpoint}, "mot": {self.mot:.2f}, "roll": {angle}'
