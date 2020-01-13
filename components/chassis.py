from wpilib.drive import DifferentialDrive
from ctre import WPI_TalonSRX, FeedbackDevice
from navx import AHRS


class Chassis:
    left_motor_slave: WPI_TalonSRX
    right_motor_slave: WPI_TalonSRX
    right_motor_master: WPI_TalonSRX
    left_motor_master: WPI_TalonSRX
    navx: AHRS

    def setup(self):
        self.left_motor_slave.follow(self.left_motor_master)
        self.left_motor_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder)
        self.left_motor_master.configVoltageCompSaturation(11)
        self.left_motor_master.enableVoltageCompensation(True)
        self.left_motor_slave.setInverted(False)
        self.left_motor_master.setInverted(False)

        self.right_motor_slave.follow(self.right_motor_master)
        self.right_motor_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder)
        self.right_motor_master.configVoltageCompSaturation(11)
        self.right_motor_master.enableVoltageCompensation(True)
        self.right_motor_slave.setInverted(False)
        self.right_motor_master.setInverted(False)

        self.diffrential_drive = DifferentialDrive(self.left_motor_master, self.right_motor_master)

        self.y_speed = 0
        self.z_speed = 0

    def get_angle(self):
        return self.navx.getAngle()

    def execute(self):
        self.diffrential_drive.arcadeDrive(self.y_speed, self.z_speed)

    def set_speed(self, y_speed, z_speed):
        self.y_speed = y_speed
        self.z_speed = z_speed

    def get_speed(self):
        return self.y_speed, self.z_speed

    def get_left_encoder(self):
        return self.left_motor_master.getSelectedSensorPosition()

    def get_right_encoder(self):
        return self.right_motor_master.getSelectedSensorPosition()

    def reset_encoders(self):
        self.left_motor_master.setSelectedSensorPosition(0)
        self.right_motor_master.setSelectedSensorPosition(0)

    def reset_gyro(self):
        self.navx.zeroYaw()

    def set_motors_values(self, left_speed, right_speed):
        self.right_motor_master.set(right_speed)
        self.left_motor_master.set(left_speed)
