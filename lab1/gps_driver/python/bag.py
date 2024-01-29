import rosbag
bag = rosbag.Bag('/home/sri/catkin_ws/src/gps_driver/scripts/gnss_data.bag')
for topic, msg, t in bag.read_messages(topics=['chatter']):
    print(msg)
bag.close()
