import rosbag
bag = rosbag.Bag('/home/sri/imu/src/data/imu_sri.bag')
for topic, msg, t in bag.read_messages(topics=['/imu']):
    print(msg)
bag.close()
