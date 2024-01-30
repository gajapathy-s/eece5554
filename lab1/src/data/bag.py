import rosbag
bag = rosbag.Bag('/home/sri/gnss/src/data/gps_occ.bag')
for topic, msg, t in bag.read_messages(topics=['chatter']):
    print(msg)
bag.close()
