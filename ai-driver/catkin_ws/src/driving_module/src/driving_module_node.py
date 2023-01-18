#!/usr/bin/env python


import os
from stat import ST_CTIME, ST_MODE, S_ISREG

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time
from driving_module.utils import get_model_by_type
from edgecar_msgs.msg import UpdateAi, WheelsCmdStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage

from scanner import Scanner

import config as cfg
import masks as masks

'''
    Example implementation of a simple CV algorithm.
    Some functions in here are not used for this example, but _could_ be used when using a ML model.
'''

class DrivingModule():
    def __init__(self):
        rospy.loginfo('[{}]ctor DrivingModule'.format(rospy.get_name()))
        self.vehicle_name = rospy.get_param("~veh", "edgecar")
        self.update_model_subscription_name = "/{}/ai/update_model".format(self.vehicle_name)
        self.start_driving_subscription_name = "/{}/ai/start_driving".format(self.vehicle_name)
        self.image_subscription_name = '/{}/camera_node/image/compressed'.format(self.vehicle_name)
        self.pub_car_cmd_name = '/{}/wheels_driver_node/wheels_cmd'.format(self.vehicle_name)

        rospy.loginfo("[{}] Starting with vehicle_name {}".format(rospy.get_name(), self.vehicle_name))
        rospy.loginfo("[{}] Subscribing to {}, {}".format(rospy.get_name(), self.update_model_subscription_name,
                                                          self.start_driving_subscription_name))
        self.sub_update_model = rospy.Subscriber(self.update_model_subscription_name, UpdateAi, self.update_ai_callback)
        self.sub_start_using_ai = rospy.Subscriber(self.start_driving_subscription_name, Bool, self.start_using_ai)

        self.pub_car_cmd = rospy.Publisher(self.pub_car_cmd_name, WheelsCmdStamped, queue_size=1)
        self.wheels_cmd = WheelsCmdStamped()

        self.image_subscriber = None  # will be used later on when we start using AI
        self.running = False
        self.kl = None
        
        self.scanner = Scanner()

        self.counter = 0

        self.model_name_to_use = self.get_initial_model_name()
        rospy.loginfo("[{}] Model name to use: {}".format(rospy.get_name(), self.model_name_to_use))
        self.bridge = CvBridge()

    def get_initial_model_name(self, dirpath='/data'):
        entries = (os.path.join(dirpath, fn) for fn in os.listdir(dirpath))
        entries = ((os.stat(path), path) for path in entries)

        # leave only regular files, insert creation date
        entries = ((stat[ST_CTIME], path) for stat, path in entries if S_ISREG(stat[ST_MODE]))
        rospy.loginfo("[{}] Current list with models : {}".format(rospy.get_name(), entries))
        list_from_entries = list(sorted(entries))
        if len(list_from_entries) > 0:
            return list_from_entries[-1][1]
        return None

    def start_using_ai(self, data):
        rospy.loginfo("START USING AI")
        if data.data:
            rospy.loginfo("[{}] Start Driving making use of CV AI : {}".format(rospy.get_name(), data.data))
            self.running = True
            self.image_subscriber = rospy.Subscriber(self.image_subscription_name, CompressedImage, self.process_img)
            rospy.loginfo("[{}] Subscribed to image topic".format(rospy.get_name()))
        elif not data.data:
            self.stop_running()
            
    '''
        Naive algorithm, scans for borders, steers away from nearest one.
        Fixed speed.
    '''
    def process_img(self, data):
        if data:
            try:
                DEFAULT_ANGLE = cfg.ANGLE
                DEFAULT_THROTTLE = cfg.THROTTLE

                img = self.bridge.compressed_imgmsg_to_cv2(data)
                hsv_data = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
               
                mask_bounds = masks.limits[cfg.LOCATION]
                mask = cv2.inRange(hsv_data, np.array(mask_bounds[0][0]), np.array(mask_bounds[0][1]))
                mask2 = cv2.inRange(hsv_data, np.array(mask_bounds[1][0]), np.array(mask_bounds[1][1]))
        
                total_mask = mask | mask2
                total_mask = cv2.erode(total_mask, (3, 3), 2)
                total_mask = cv2.dilate(total_mask, (3, 3), 2)
                
                image_result = cv2.bitwise_and(img, img, mask=total_mask)
                
                self.scanner.set_image(total_mask)
                (target, absdir, left, right) = self.scanner.direction()

                angle = DEFAULT_ANGLE * absdir
                throttle = DEFAULT_THROTTLE

                self.wheels_cmd.header.stamp = data.header.stamp
                self.wheels_cmd.velocity = throttle
                self.wheels_cmd.rotation = angle
                self.pub_car_cmd.publish(self.wheels_cmd)

            except CvBridgeError as e:
                rospy.logerr("CvBridgeError : ", e)

    def image_callback(self, data):
        if data:
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                cv_image = cv2.resize(cv_image, (160, 120))
                outputs = self.kl.run(cv_image)

                self.wheels_cmd.header.stamp = data.header.stamp
                self.wheels_cmd.velocity = outputs[0]
                self.wheels_cmd.rotation = outputs[1]
                self.pub_car_cmd.publish(self.wheels_cmd)

                if self.counter % 20 == 0:
                    rospy.loginfo("[{}] outputs : {}".format(rospy.get_name(), outputs))
                    self.counter = 0
                self.counter += 1
            except CvBridgeError as e:
                rospy.logerr("error cvbridge stuff : ", e)

    def update_ai_callback(self, data):
        rospy.loginfo("[{}] Update Model : {}".format(rospy.get_name(), data))
        was_running = self.running
        if self.running:
            self.stop_running()
        self.use_new_model(data.model_name)

        if was_running:
            temp_restart = Bool()
            temp_restart.data = True
            was_running = None
            self.start_using_ai(temp_restart)

    def use_new_model(self, model_name):
        rospy.loginfo("[{}] Use new model '{}'".format(rospy.get_name(), model_name))
        self.model_name_to_use = "/data/{}".format(model_name)
        self.kl = get_model_by_type()
        self.load_model(self.kl, self.model_name_to_use)

        rospy.loginfo(("[{}] Model loaded".format(rospy.get_name())))

    def stop_running(self):
        rospy.loginfo("[{} Must stop running AI container".format(rospy.get_name()))
        if not self.image_subscriber is None:
            self.image_subscriber.unregister()
            self.image_subscriber = None
            rospy.logdebug("[{}] Finished unsubscribing from image stream".format(rospy.get_name()))
        self.running = False

    def on_shutdown(self):
        rospy.loginfo('[DrivingModule] onShutdown ...')

    def load_model(self, kl, model_path):
        start = time.time()
        try:
            rospy.loginfo('[{}] loading model {}'.format(rospy.get_name(), model_path))
            kl.load(model_path)
            rospy.loginfo('[{}] finished loading in {} sec.'.format(rospy.get_name(), str(time.time() - start)))
        except Exception as e:
            print(e)
            print('ERR>> problems loading model', model_path)


if __name__ == '__main__':
    rospy.init_node('driving_node', anonymous=False)
    driving_module = DrivingModule()
    rospy.on_shutdown(driving_module.on_shutdown)
    rospy.spin()
