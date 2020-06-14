#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from edgecar_msgs.msg import BoolStamped, WheelsCmdStamped

 
class JoystickMapper():
    def __init__(self):
        self.node_name = rospy.get_name()
        self.vehicle_name = rospy.get_param('~veh', 'edgecar')
        rospy.loginfo("[%s] Initializing : %s" % (self.node_name, self.vehicle_name))

        self.emergency_triggered = False
        self.inverse_throttle = rospy.get_param('~inverse_throttle', True)
        self.inverse_steering = rospy.get_param('~inverse_steering', False)
        #  publish
        self.pub_emergency_stop = rospy.Publisher("/%s/wheels_driver_node/emergency_stop" % self.vehicle_name,
                                                  BoolStamped, queue_size=1)
        self.pub_car_cmd = rospy.Publisher('/%s/wheels_driver_node/wheels_cmd' % self.vehicle_name, WheelsCmdStamped, queue_size=1)

        # subscribe to joystick ...
        self.joy_sub = rospy.Subscriber('/%s/joy' % self.vehicle_name, Joy, self.joy_callback, queue_size=1)

    def joy_callback(self, data):
        self.joy_message = data
        self.process_steering(data)
        self.process_buttons(data)
        rospy.logdebug('[JoystickMapper] Finished processing joystick')

    def process_steering(self, joy_msg):
        vel = joy_msg.axes[1]
        angle = joy_msg.axes[0]

        if self.inverse_steering:
            angle = angle * -1.0
        if self.inverse_throttle:
            vel = vel * -1.0

        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = joy_msg.header.stamp
        wheels_cmd.velocity = vel
        wheels_cmd.rotation = angle

        self.pub_car_cmd.publish(wheels_cmd)

    def process_buttons(self, joy_msg):
        if joy_msg.buttons[0] == 0:
            #  emergency button !!
            if not self.emergency_triggered:
                emergency_stop_msg = BoolStamped()
                emergency_stop_msg.header.stamp = joy_msg.header.stamp
                self.emergency_triggered = True
                rospy.loginfo('[JoystickMapper] Emergency trigered')
                emergency_stop_msg.data = self.emergency_triggered
                self.pub_emergency_stop.publish(emergency_stop_msg)
        elif joy_msg.buttons[0] == 1:
            # emergency released
            if self.emergency_triggered:
                emergency_stop_msg = BoolStamped()
                emergency_stop_msg.header.stamp = joy_msg.header.stamp
                self.emergency_triggered = False
                rospy.loginfo('[JoystickMapper] Emergency Released')
                emergency_stop_msg.data = self.emergency_triggered
                self.pub_emergency_stop.publish(emergency_stop_msg)

    def on_shutdown(self):
        rospy.loginfo('[%s] on_shutdown ' % rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('joy_mapper_node', anonymous=False)
    js_mapper = JoystickMapper()
    rospy.on_shutdown(js_mapper.on_shutdown)
    rospy.spin()
