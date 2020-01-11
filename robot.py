from components.chassis import Chassis
from components.pidcontroller import PIDControl
from magicbot import MagicRobot
from wpilib.joystick import Joystick
from ctre import WPI_TalonSRX
import wpilib
from navx import AHRS

class Robot(MagicRobot):
    pidcontroller: PIDControl
    chassis: Chassis
    joystick: Joystick

    def createObjects(self):
        self.chassis_left_motor_master = WPI_TalonSRX(1)
        self.chassis_left_motor_slave = WPI_TalonSRX(2)
        self.chassis_right_motor_master = WPI_TalonSRX(3)
        self.chassis_right_motor_slave = WPI_TalonSRX(4)
        self.chassis_navx = AHRS.create_spi()
        self.joystick = Joystick(0)

    def teleopPeriodic(self):
        self.chassis.set_speed(-self.joystick.getY(), -self.joystick.getZ())


if __name__ == '__main__':
    wpilib.run(Robot)
