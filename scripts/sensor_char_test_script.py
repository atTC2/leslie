import rospy
from std_msgs.msg import String
import json
import time

distro_size = [400]
importance = [5]
std_dev = [70]
angle_split = [7]
distance = [0.5, 1, 1.5, 2, 0.5, 1, 1.5, 2, 0.5, 1, 1.5, 2]

back_home = True

if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)

def state_callback(data):
    global back_home
    data_json = json.loads(data.data)
    if data_json['id'] == 'FAKE_ARRIVE':
        back_home = True
        print data_json['data']['dist_diff'],',', data_json['data']['angle_diff']


rospy.init_node('sensor_char_test', anonymous=True)
pub = rospy.Publisher('/state', String, queue_size=10)
rospy.Subscriber('/state', String, state_callback, queue_size=10)

time.sleep(5)

index = 0

for size in distro_size:
    for imp in importance:
        for dev in std_dev:
            for angle in angle_split:
                for d in distance:
                    data = {'distro_size': size,
                            'importance': imp,
                            'std_dev_evidence': dev,
                            'angle_split': angle,
                            'colour': [0, 0, 255],
                            'distance': d}
                    index += 1
                    print index,',',size,',',imp,',',dev,',',angle,',', distance,','
                    back_home = False
                    state_data = {'id': 'FAKE_ALARM', 'data': data}
                    pub.publish(json.dumps(state_data))

                    while not back_home:
                        time.sleep(2)
