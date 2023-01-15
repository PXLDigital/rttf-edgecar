import rospy

class PWMSteering:
    LEFT_ANGLE=-1.0
    RIGHT_ANGLE=1.0

    def __init__(self, controller=None, left_pulse=300.0, right_pulse=490.0):
        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse

    def set_angle(self, angle):
        pulse = self.map_range(angle, self.LEFT_ANGLE, self.RIGHT_ANGLE, self.left_pulse, self.right_pulse)

        rospy.logdebug('[%s] Set speed pulse to %s' % (rospy.get_name(), pulse))
        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)

    def map_range(self, x, x_MIN, x_MAX, y_MIN, y_MAX):
        x_range = x_MAX - x_MIN
        y_range = y_MAX - y_MIN
        xy_ratio = x_range * 1.0 / y_range * 1.0

        y=((x - x_MIN)/ xy_ratio + y_MIN) // 1

        return int(y)
