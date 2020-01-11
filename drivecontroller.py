from wpilib import PIDController


class DriveController:
    wheel_radius = 0.1  # meters
    encoder_ticks_per_revolution = 256 * 3*1.8

    def __init__(self, kP, kI, kD, kF, tolerance, encoder_function, update_function, direction, period, is_enabled):
        self.is_enabled = is_enabled
        self.direction = direction
        self.encoder_function = encoder_function
        self.motor_output = 0

        self.pid_controller = PIDController(kP, kI, kD, kF, source=self, output=self, period=period)
        #self.pid_controller.controlLoop._thread.setDaemon(True)
        self.pid_controller.setInputRange(-5, 5)

        self.pid_controller.setOutputRange(-1.0, 1.0)
        self.pid_controller.setAbsoluteTolerance(tolerance)
        self.update_function = update_function
        self.pid_controller.setContinuous(True)

    def start(self, setpoint):
        self.setpoint = setpoint
        self.pid_controller.setSetpoint(setpoint)
        self.pid_controller.enable()

    def pidWrite(self, output):
        print("output voltage",output)
        # print("distance for cotroller", self.pid_controller.getError())
        self.pid_controller.setSetpoint(self.setpoint)
        self.update_function(self.direction * output)

    def is_finished(self):
        return self.pid_controller.onTarget()

    def close(self):
        self.pid_controller.close()

    def pidGet(self):
        distance_in_encoder = self.encoder_function()
        distance_in_meters = (distance_in_encoder / self.encoder_ticks_per_revolution) * self.wheel_radius
        print("dist", distance_in_meters)
        return distance_in_meters

    def getPIDSourceType(self):
        return "meter"

    def getPIDOutputType(self):
        return "output"