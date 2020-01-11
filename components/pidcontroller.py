from magicbot import StateMachine, state
from components.chassis import Chassis
from drivecontroller import DriveController
from ioeholder import IOEncoderHolder, IOGyroHolder
from wpilib import PIDController, Notifier


class PIDControl(StateMachine):
    enc_ticks_per_rev = 256
    chassis: Chassis
    left_speed = 0
    right_speed = 0
    p = 0
    i = 0
    d = 0
    f = 0
    tolerance = 0
    period = 0
    wheel_diameter = 0
    input_range = 0
    output_range = 0
    distance_set_point = 0
    finished = False
    angle = 0

    def setup(self):
        self.motor_updater = Notifier(self.update_motors)
        self.final_setup()

    def setup_values(self, p, i, d, f, tolerance, period, wheel_diameter, input_range, output_range, distance):
        self.p = p
        self.i = i
        self.d = d
        self.f = f
        self.tolerance = tolerance
        self.period = period
        self.wheel_diameter = wheel_diameter
        self.input_range = input_range
        self.output_range = output_range
        self.distance_set_point = distance

    def final_setup(self):
        self.left_holder = IOEncoderHolder(self.enc_ticks_per_rev, self.wheel_diameter, self.chassis.get_left_encoder,
                                           self.left_output)
        self.left_controller = PIDController(self.p, self.i, self.d, self.f, source=self.left_holder,
                                             output=self.left_holder, period=self.period)
        self.left_controller.setContinuous(True)
        self.left_controller.setOutputRange(-self.output_range, self.output_range)
        self.left_controller.setInputRange(-self.input_range, self.input_range)
        self.left_controller.setAbsoluteTolerance(self.tolerance)
        self.right_holder = IOEncoderHolder(self.enc_ticks_per_rev, self.wheel_diameter, self.chassis.get_right_encoder,
                                            self.right_output)
        self.right_controller = PIDController(self.p, self.i, self.d, self.f, source=self.right_holder,
                                              output=self.right_holder, period=self.period)
        self.right_controller.setContinuous(True)
        self.right_controller.setOutputRange(-self.output_range, self.output_range)
        self.right_controller.setInputRange(-self.input_range, self.input_range)
        self.right_controller.setAbsoluteTolerance(self.tolerance)
        self.angle_holder = IOGyroHolder(self.chassis.get_angle, self.gyro_output)
        self.gyro_controller = PIDController(self.p, self.i, self.d)
    def left_output(self, speed):
        self.left_speed = speed

    def update_motors(self):
        self.chassis.set_motors_values(self.left_speed - self.angle, self.right_speed + self.angle)

    def right_output(self, speed):
        self.right_speed = speed

    def gyro_output(self, angle):
        self.angle = angle

    def is_finished(self):
        return self.left_controller.onTarget() and self.right_controller.onTarget()

    def reset(self):
        self.i = 0
        self.p = 0
        self.d = 0
        self.input_range = 0
        self.distance_set_point = 0

    def initialize(self):
        self.chassis.reset_encoders()
        self.chassis.reset_gyro()
        self.left_controller.setSetpoint(self.distance_set_point)
        self.right_controller.setSetpoint(self.distance_set_point)
        self.motor_updater.startPeriodic(self.period)

    @state(first=True)
    def drive(self, initial_call):
        if initial_call:
            self.initialize()
        if self.is_finished:
            self.finished = True
            self.done()
