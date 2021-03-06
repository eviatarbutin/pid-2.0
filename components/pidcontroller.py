from components.chassis import Chassis
from ioeholder import IOEncoderHolder, IOGyroHolder
from magicbot import StateMachine, state
from wpilib import PIDController, Notifier


class PIDControl(StateMachine):
    chassis: Chassis
    enc_ticks_per_rev = 256 * 1.8 * 3 * 4
    p = 0.1
    i = 0
    d = 0
    f = 0
    p_gyro = 0.1
    i_gyro = 0
    d_gyro = 0
    gyro_tolerance = 10
    gyro_input_range = 180
    left_speed = 0
    right_speed = 0
    tolerance = 0.05
    period = 0.02
    wheel_diameter = 0.1
    input_range = 15
    output_range = 1
    distance_set_point = 9
    angle_set_point = 90
    gyro = 0
    finished = False

    def setup(self):
        self.motor_updater = Notifier(self.update_motors)
        self.final_setup()

    def final_setup(self):
        self.left_holder = IOEncoderHolder(self.enc_ticks_per_rev, self.wheel_diameter, self.chassis.get_left_encoder,
                                           self.left_output)
        self.left_controller = PIDController(self.p, self.i, self.d, self.f, source=self.left_holder,
                                             output=self.left_holder, period=self.period)
        self.left_controller.setInputRange(-self.input_range, self.input_range)
        self.left_controller.setOutputRange(-self.output_range, self.output_range)
        self.left_controller.setAbsoluteTolerance(self.tolerance)

        self.right_holder = IOEncoderHolder(self.enc_ticks_per_rev, self.wheel_diameter, self.chassis.get_right_encoder,
                                            self.right_output)
        self.right_controller = PIDController(self.p, self.i, self.d, self.f, source=self.right_holder,
                                              output=self.right_holder, period=self.period)
        self.right_controller.setInputRange(-self.input_range, self.input_range)
        self.right_controller.setOutputRange(-self.output_range, self.output_range)
        self.right_controller.setAbsoluteTolerance(self.tolerance)

        self.angle_holder = IOGyroHolder(self.chassis.get_angle, self.gyro_output)
        self.gyro_controller = PIDController(self.p_gyro, self.i_gyro, self.d_gyro, self.f, source=self.angle_holder,
                                             output=self.angle_holder, period=self.period)
        self.gyro_controller.setInputRange(0, 360)
        self.gyro_controller.setOutputRange(-self.output_range, self.output_range)
        self.gyro_controller.setAbsoluteTolerance(self.gyro_tolerance)

    def left_output(self, speed):
        self.left_speed = speed

    def update_motors(self):
        self.chassis.set_motors_values(self.left_speed - self.gyro, self.right_speed + self.gyro)

    def right_output(self, speed):
        self.right_speed = speed

    def gyro_output(self, angle):
        self.gyro = angle

    def is_finished(self):
        return self.left_controller.onTarget() and self.right_controller.onTarget() and self.gyro_controller.onTarget()

    def reset(self):
        self.input_range = 0
        self.distance_set_point = 0

        self.angle_set_point = 0
        self.gyro_input_range = 0

    def initialize(self):
        self.chassis.reset_encoders()
        self.chassis.reset_gyro()

        self.left_controller.setSetpoint(self.distance_set_point)
        self.right_controller.setSetpoint(self.distance_set_point)
        self.gyro_controller.setSetpoint(self.angle_set_point)

        self.motor_updater.startPeriodic(self.period)
        self.left_controller.enable()
        self.right_controller.enable()
        self.gyro_controller.enable()

    def close_threads(self):
        self.gyro_controller.close()
        self.left_controller.close()
        self.right_controller.close()
        self.motor_updater.close()

    @state(first=True)
    def drive(self, initial_call):
        if initial_call:
            self.initialize()
        elif self.is_finished():
            self.close_threads()
            self.finished = True
            self.done()
