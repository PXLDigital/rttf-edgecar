#!/usr/bin/env python

import datetime
import rospy
from azure.storage.blob.blockblobservice import BlockBlobService
import message_filters
from sensor_msgs.msg import CompressedImage, Joy
from std_msgs.msg import Bool
from edgecar_msgs.msg import UpdateAi
import os
from os import listdir
from os.path import isfile, join, isdir
import shutil


class StorageTub():
    def __init__(self):
        rospy.loginfo('[storage] constructor %s' % rospy.get_name())

        self.must_store = False
        self.path = rospy.get_param('~data_path', 'data')
        self.vehicle_name = rospy.get_param('~veh', 'edgecar')

        rospy.loginfo("[storage] using vehicle name {}".format(self.vehicle_name))

        self.container_name = rospy.get_param('~container_name', 'cardrivers')
       
        self.container_name_models = "models"

        self.ensure_container()

        self.start_recording_sub = rospy.Subscriber('/%s/storage/start_recording' % self.vehicle_name, Bool,
                                                    self.start_recording_callback)
                                                     

        self.ai_update_model_sub = rospy.Subscriber('/{}/storage/update_model'.format(self.vehicle_name), UpdateAi,
                                                    self.on_update_ai_model, queue_size=1)
        self.ai_update_model_pub = rospy.Publisher('/{}/ai/update_model'.format(self.vehicle_name), UpdateAi,
                                                   queue_size=1)

        rospy.loginfo('[storage] Subscribing an ApproximateTimeSynchronizer to Images and Joystick')
        self.image_sub = message_filters.Subscriber('/%s/camera_node/image/compressed' % self.vehicle_name,
                                                    CompressedImage)
        self.joy_sub = message_filters.Subscriber('/%s/joy' % self.vehicle_name, Joy)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.joy_sub], 10, 0.1)
        rospy.loginfo('[storage] registering an callback to te filtered messages')
        self.ts.registerCallback(self.filter_callback)

        self.tub_path = ''
        self.current_idx = 0
        rospy.loginfo('[storage] finished ctor')

    def ensure_container(self):
        if not os.path.isdir('/data/tubs'):
            os.makedirs('/data/tubs')
        rospy.loginfo('[storage] ensure container %s' % self.container_name)
       

    def next_tub_number(self):
        def get_tub_num(tub_name):
            try:
                num = int(tub_name.split('_')[1])
            except:
                num = 0
            return num

        folder_names = [name for name in os.listdir('/{}/tubs/'.format(self.path))]
        rospy.loginfo('[tub] folder names : {}'.format(folder_names))
        # folders = [str(l) for l in folder_names if l.find('/') >= 0 and l != '/']
        numbers = [get_tub_num(x) for x in folder_names]
        rospy.loginfo('[tub] number : {}'.format(numbers))
        next_number = max(numbers + [0]) + 1
        rospy.loginfo('[tub] next_number {}'.format(next_number))
        return next_number

    def create_tub_path(self):
        tub_num = self.next_tub_number()
        date = datetime.datetime.now().strftime('%y-%m-%d')
        name = '_'.join(['tub', str(tub_num), date])
        rospy.loginfo('[Tub] next tub folder: ' + name)
        return name

    def ensure_folder(self):
        rospy.loginfo('[Tub] Create new tub')
        self.tub_path = self.create_tub_path()
        self.current_idx = 0
        self.make_sure_folder_exists('/{}/tubs/{}'.format(self.path, self.tub_path))
        rospy.loginfo("[Tub] Currently using index : " + self.tub_path + ' -> ' + str(self.current_idx))

    def get_last_idx(self):
        index = self.get_index()
        return max(index)

    def get_index(self):
        index = self.block_service.list_blob_names(self.container_name)
        record_files = [f for f in index if f[:6] == 'record']

        def get_file_ix(file_name):
            try:
                name = file_name.split('.')[0]
                num = int(name.split('_')[1])
            except Exception as e:
                num = 0
            return num

        nums = [get_file_ix(f) for f in record_files]
        nums = sorted(nums)
        return nums

    def store_image_data(self, im, index_to_use):
        data = im.data
 
        file_name = "%s_cam_image_array.jpg" % index_to_use
        blob_name = "/%s/tubs/%s/%s" % (self.path, self.tub_path, file_name)

        rospy.loginfo("[tub] blob_name : {}, container_name = {}".format(blob_name, self.container_name))


        with open(blob_name, 'wb') as f:
            f.write(data)
        return blob_name

    def store_tub_data(self, axes, image_name, index_to_use):
        file_name = "record_%s.json" % index_to_use
        data = '{"user/angle" : %s, "user/throttle" : %s, "cam/image_array":"%s"}' % (axes[0], axes[1], image_name)
        blog_name = "/%s/tubs/%s/%s" % (self.path, self.tub_path, file_name)

        with open(blog_name, 'wb') as f:
            f.write(data)
        
        rospy.loginfo("[tub] blob_name : {}, container_name = {}".format(blog_name, self.container_name))
        
    def create_finished_tag(self):
        blob_name = "/%s/tubs/%s/%s" % (self.path, self.tub_path, "finished.txt")
        data = '{"finished" : true}'
        with open(blob_name, 'wb') as f:
            f.write(data)
        
    def filter_callback(self, im, j):
        rospy.logdebug("[{}] filter_callback => {}".format(rospy.get_name(),self.must_store))
        if self.must_store: 
            axes = j.axes
            self.current_idx += 1
            image_name = self.store_image_data(im, self.current_idx)
            self.store_tub_data(axes, image_name, self.current_idx)

    def on_update_ai_model(self, msg):
        rospy.loginfo("[{}] Download model {}".format(rospy.get_name(), msg.model_name))
        # TODO => fix model service, not needed ... 
        self.model_block_service.get_blob_to_path(self.container_name_models, msg.model_name,
                                                  '/data/{}'.format(msg.model_name))
        rospy.loginfo("[{}] Model downloaded. Trigger Ai to reload".format(rospy.get_name()))
        self.trigger_ai(msg)

    def trigger_ai(self, msg):
        self.ai_update_model_pub.publish(msg)

    def start_recording_callback(self, msg):
        rospy.loginfo('[Tub] recording callback : ' + str(msg.data))
        if msg.data:
            self.ensure_folder()
        elif not msg.data:
            rospy.loginfo('[tub] => store finished tag')

            self.create_finished_tag()

        self.must_store = msg.data

    def traverse(self, root=None):
        if root is None:
            root = '/data/tubs/'
        files = [ join(root,f) for f in listdir(root) if isfile(join(root,f))]
        dirs = [ d for d in listdir(root) if isdir(join(root,d))]
        for d in dirs:
            files_in_d = self.traverse(join(root,d))
            if files_in_d:
                for f in files_in_d:
                    files.append(join(root,f))
        return files
    
    def start_sync_callback(self, msg):
        rospy.loginfo('[tub] Start synchronisation of tubs : {}'.format(msg.data))
        if msg.data:
            rospy.loginfo('[{}] traverse all folders and sync them... delete them afterwards'.format(rospy.get_name()))
            all_files_to_sync = self.traverse()
            self.sync_files_from_disk(all_files_to_sync)
            self.remove_folder('/data/tubs')
            os.mkdir('/data/tubs')
    
    def remove_folder(self, folder_name):
        shutil.rmtree(folder_name)

    def remove_file_from_disk(self, f):
        if os.path.isfile(f):
            os.remove(f)

  
    def onShutdown(self):
        rospy.loginfo('[Storage] onShutdown')

    def make_sure_folder_exists(self, path_to_check):
        if not os.path.exists(path_to_check):
            os.makedirs(path_to_check)


if __name__ == '__main__':
    rospy.init_node('storage_node', anonymous=False)
    storage_node = StorageTub()
    rospy.on_shutdown(storage_node.onShutdown)
    rospy.spin()
