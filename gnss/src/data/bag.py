import rosbag
bag = rosbag.Bag('/home/sri/gnss/src/data/occludedRTK.bag')
for topic, msg, t in bag.read_messages(topics=['rtk_gnss']):
    print(msg)
bag.close()
