import rosbag
bag = rosbag.Bag('circle_2024-03-27-14-05-03 1.bag')
for topic, msg, t in bag.read_messages(topics=['/gps']):
    print(msg)
bag.close()
