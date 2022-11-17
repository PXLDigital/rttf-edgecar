import time

import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

talker = roslibpy.Topic(client, '/master/ai/start_driving', 'std_msgs/Bool', None)

while client.is_connected:
    talker.publish(roslibpy.Message({'data': False}))
    print('Sending message...')
    time.sleep(1)

talker.unadvertise()

client.terminate()