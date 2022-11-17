from __future__ import print_function
import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

#listener = roslibpy.Topic(client, '/master/ai/start_driving', 'std_msgs/Bool', None)
listener = roslibpy.Topic(client, '/master/camera_node/image/compressed', 'sensor_msgs/CompressedImage')
listener.subscribe(lambda message: print('Heared talking: ' + str(message['data'])))

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()