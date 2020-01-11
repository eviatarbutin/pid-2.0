import math


class IOEncoderHolder:
    def __init__(self, encoder_ticks_per_revolution, wheel_diameter, encoder_func, motor_func):
        self.ticks = encoder_ticks_per_revolution
        self.circumference = wheel_diameter * math.pi
        self.get_encoder = encoder_func
        self.update_function = motor_func

    def pidWrite(self, output):
        # print("output voltage", output)
        self.update_function(output)

    def pidGet(self):
        distance_in_encoder = self.get_encoder()
        distance_in_meters = (distance_in_encoder / self.ticks) * self.circumference
        # print("dist", distance_in_meters)
        return distance_in_meters

    def getPIDSourceType(self):
        return 0


class IOGyroHolder:
    def __init__(self, gyro_func, motor_func):
        self.get_angle = gyro_func
        self.update_function = motor_func
        self.trackwidth = 0.813
        self.f = 0.68 / self.trackwidth

    def pidWrite(self, output):
        # print("turn voltage", output)
        output = output * (1 + 2 * self.f) * (self.trackwidth / 2)
        self.update_function(output)

    def pidGet(self):
        angle = self.get_angle()
        if angle < 0:
            # angle = angle%360
            angle = 360 + angle
        angle = angle % 360
        # print("angle", angle)
        return angle

    def getPIDSourceType(self):
        return 1
