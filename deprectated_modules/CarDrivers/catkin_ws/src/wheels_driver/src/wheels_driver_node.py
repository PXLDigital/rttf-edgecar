#!/usr/bin/env python
import os
import rospy
from edgecar_msgs.msg import WheelsCmdStamped, BoolStamped
#TODO: Nicky => fix wheels driver to use 
from wheels_driver.RcDriver import RcDriver
from PCA9685 import PCA9685_Driver, PWMSteering, PWMThrottle
#from wheels_driver.dagu_wheels_driver import DaguWheelsDriver
import numpy as np
 
class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False

        # Parameters for maximal turning radius
        self.use_rad_lim        =   self.setupParam("~use_rad_lim", False)
        self.min_rad            =   self.setupParam("~min_rad", 0.08)
        self.wheel_distance     =   self.setupParam("~wheel_distance", 0.103)

        self.steering_channel     =   self.setupParam('~steering_channel',1)
        self.steering_left_pwm    =   self.setupParam('~steering_left_pwm',460)
        self.steering_right_pwm   =   self.setupParam('~steering_right_pwm',290)

        
        self.throttle_channel       =   self.setupParam('~throttle_channel', 0)
        self.throttle_forward_pwm   =   self.setupParam('~throttle_forward_pwm',500)
        self.throttle_stopped_pwm   =   self.setupParam('~throttle_stopped_pwm',370)
        self.throttle_reverse_pwm   =   self.setupParam('~throttle_reverse_pwm', 220)

        self.PCA9685_I2C_ADDRESS =  self.setupParam("~PCA9685_I2C_ADDRESS", 0x40)
        self.PCA9685_I2C_BUSNUM =   os.environ.get('I2C_BUSNUM', 1)
        rospy.loginfo('Current busnum for I2C = {}'.format(self.PCA9685_I2C_BUSNUM))

        rospy.loginfo("Create steering controller")
        self.steering_controller = PCA9685_Driver(self.steering_channel, self.PCA9685_I2C_ADDRESS, busnum=self.PCA9685_I2C_BUSNUM)
        self.steering = PWMSteering(controller = self.steering_controller, 
                                                left_pulse=self.steering_left_pwm,
                                                right_pulse=self.steering_right_pwm)
        
        self.throttle_controller = PCA9685_Driver(self.throttle_channel, self.PCA9685_I2C_ADDRESS, busnum=self.PCA9685_I2C_BUSNUM)
        self.throttle = PWMThrottle(controller=self.throttle_controller,
                                                max_pulse=self.throttle_forward_pwm,
                                                zero_pulse=self.throttle_stopped_pwm,
                                                min_pulse=self.throttle_reverse_pwm)
        
        #add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed",WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        # Done 
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)
        
        
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)


        self.sub_rad_lim = rospy.Subscriber("~radius_limit", BoolStamped, self.cbRadLimit, queue_size=1)

    #    self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updateParams(self,event):
        rospy.loginfo("Update params ")
        # self.use_rad_lim        =   rospy.get_param("~use_rad_lim")
        # self.min_rad            =   rospy.get_param("~min_rad")
        # self.wheel_distance     =   rospy.get_param("~wheel_distance")

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.throttle.set_speed(0)
            # self.driver.setWheelsSpeed(left=0.0,right=0.0)
            return
        #enable debugging
        rospy.logdebug('[%s] got wheels command %s' % (rospy.get_name(), msg))
        # Check if radius limitation is enabled
        #if (self.use_rad_lim and (msg.vel_left != 0 or msg.vel_right != 0)):
        #    self.checkAndAdjustRadius(msg)

        # TODO nicky
        self.throttle.set_speed(msg.velocity)
        self.steering.set_angle(msg.rotation)
        #self.driver.setWheelsSpeed(left=msg.vel_left,right=msg.vel_right)
        # Put the wheel commands in a message and publish
        # self.msg_wheels_cmd.header = msg.header
        # # Record the time the command was given to the wheels_driver
        # self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
        # self.msg_wheels_cmd.vel_left = msg.vel_left
        # self.msg_wheels_cmd.vel_right = msg.vel_right
        # self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbRadLimit(self, msg):
        rospy.set_param("~use_rad_lim", msg.data)
        self.use_rad_lim = msg.data


    def cbEStop(self,msg):
        rospy.loginfo('[wheels_driver_node] cbEstop =>  %s' % msg.data)
        self.estop=msg.data
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated" % rospy.get_name())
        else:
            rospy.loginfo("[%s] Emergency Stop Released" % rospy.get_name())

    def on_shutdown(self):
        self.driver.setWheelsSpeed(left=0.0,right=0.0)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
