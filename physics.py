from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
from math import pi


class PhysicsEngine(object):
    """
       Simulates a 4-wheel robot using Tank Drive joystick control
    """
    reset = False

    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        self.physics_controller.add_device_gyro_channel("navxmxp_spi_4_angle")
        self.wheel_radius=0.075
        self.encoder_ticks_per_revolution = 256

        # Change these parameters to fit your robot!
        bumper_width = 0 * units.inch
        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,  # motor configuration
            110 * units.lbs,  # robot mass
            6.601,  # drivetrain gear ratio
            2,  # motors per side
            26.7716535 * units.inch,  # robot wheelbase
            (26.7716535 + 1) * units.inch + bumper_width * 2,
            # robot width
            32 * units.inch + bumper_width * 2,  # robot length
            6 * units.inch,  # wheel diameter
        )

    def update_sim(self, hal_data, now, tm_diff):
        """
            Called when the simulation parameters for the program need to be
            updated.

            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        """For drivetrain simulation"""
        lm_motor = hal_data['CAN'][2]['value']
        rm_motor = hal_data['CAN'][4]['value']

        x, y, angle = self.drivetrain.get_distance(lm_motor, rm_motor, tm_diff)
        self.physics_controller.distance_drive(x, y, angle)

        wheel_diameter = self.wheel_radius * 2 * pi
        ticks_per_meter = self.encoder_ticks_per_revolution / wheel_diameter

        hal_data['CAN'][2]['quad_position'] = int(self.drivetrain.l_position * ticks_per_meter)
        hal_data['CAN'][4]['quad_position'] = int(self.drivetrain.r_position * ticks_per_meter)
        hal_data.setdefault('custom', {})['left encoder'] = hal_data['CAN'][2]['quad_position']
        hal_data.setdefault('custom', {})['right encoder'] = hal_data['CAN'][4]['quad_position']


        def reset():
            hal_data['CAN'][2]['quad_position'] = 0
            hal_data['CAN'][4]['quad_position'] = 0
            hal_data['robot']['navxmxp_spi_4_angle'] = 0
            PhysicsEngine.reset = False

        if PhysicsEngine.reset:
            print("reset")
            reset()
        """
        # TODO convert velocity measurement units
        hal_data['CAN'][2]['quad_velocity'] = self.drivetrain.l_velocity
        hal_data['CAN'][4]['quad_velocity'] = self.drivetrain.r_velocity
        """
