from simple_pid import PID
from encoders import Encoders
from mpu import mpu
from dual_tb9051ftg_rpi import motors, MAX_SPEED

class pid:
    def __init__(self, timer_tick):

        sample_time = timer_tick / 1000.
        self.Kp = 5
        self.Ki = 0.2
        self.Kd = 0.1
        self.left_setpoint = 0
        self.right_setpoint = 0
        self.ec_left_last = 0
        self.ec_right_last = 0

        self.encoders = Encoders()
        self.pid_left = PID(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, setpoint=self.left_setpoint, sample_time=sample_time, output_limits=(-MAX_SPEED,MAX_SPEED))
        self.pid_right = PID(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, setpoint=self.right_setpoint, sample_time=sample_time, output_limits=(-MAX_SPEED,MAX_SPEED))

        self.mpu = mpu()
        self.motors = motors

    def update(self):
        self.pid_left.setpoint = self.left_setpoint
        self.pid_right.setpoint = self.right_setpoint
        self.pid_left.Kp = self.Kp
        self.pid_right.Kp = self.Kp
        self.pid_left.Ki = self.Ki
        self.pid_right.Ki = self.Ki
        self.pid_left.Kd = self.Kd
        self.pid_right.Kd = self.Kd

        self.delta_left = self.encoders.ec_left.pos - self.ec_left_last
        self.delta_right = self.encoders.ec_right.pos - self.ec_right_last
        self.ec_left_last = self.encoders.ec_left.pos
        self.ec_right_last = self.encoders.ec_right.pos
        self.mot_left = self.pid_left(self.delta_left)
        self.mot_right = self.pid_right(self.delta_right)
        
        self.motors.setSpeeds(int(self.mot_left), int(self.mot_right))

        return f'"ec_left_pos": {self.encoders.ec_left.pos},"ec_right_pos": {self.encoders.ec_right.pos},"left_setpoint": {self.left_setpoint},"right_setpoint": {self.right_setpoint},"delta_left": {self.delta_left:.2f},"delta_right": {self.delta_right:.2f},"mot_left": {self.mot_left:.2f},"mot_right": {self.mot_right:.2f}'
