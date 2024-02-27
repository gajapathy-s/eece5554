#!/usr/bin/env python3


import rospy
import rosbag
import time
import rospy
import math
from std_msgs.msg import String
from vn_driver.msg import Vectornav







def VNYMR(VNYMR_string,x,z):
    
    fields = VNYMR_string.split(',')
    title=str(fields[0])
    
    vnmsg.header.frame_id = 'imu1_frame'
    vnmsg.header.stamp.secs=x
    vnmsg.header.stamp.nsecs=((z))*(1e9)
    
    vnmsg.imu.header.frame_id = 'imu1_frame'
    vnmsg.imu.header.stamp.secs=x
    vnmsg.imu.header.stamp.nsecs=((z))*(1e9)



    vnmsg.mag_field.header.frame_id = 'imu1_frame'
    vnmsg.mag_field.header.stamp.secs=x
    vnmsg.mag_field.header.stamp.nsecs=((z))*(1e9)
    
    
    if title == "$VNYMR":

        Y=float(fields[1])
        P= float(fields[2])
        R=float(fields[3])

        pi = math.pi
        yaw=(pi/180)*Y
        pitch=(pi/180)*P
        roll=(pi/180)*R

        h_y=yaw/2
        h_p=pitch/2
        h_r=roll/2


        vnmsg.imu.orientation.w = float((math.cos(h_y) * math.cos(h_p) * math.cos(h_r)) + (math.sin(h_y) * math.sin(h_p) * math.sin(h_r)))
        vnmsg.imu.orientation.x = float((math.cos(h_y) * math.sin(h_p) * math.cos(h_r)) - (math.sin(h_y) * math.cos(h_p) * math.sin(h_r)))
        vnmsg.imu.orientation.y =float((math.cos(h_y) * math.cos(h_p) * math.sin(h_r)) + (math.sin(h_y) * math.sin(h_p) * math.cos(h_r)))
        vnmsg.imu.orientation.z = float((math.sin(h_y) * math.sin(h_p) * math.cos(h_r)) - (math.cos(h_y) * math.sin(h_p) * math.sin(h_r)))



        vnmsg.mag_field.magnetic_field.x=float(fields[4])
        vnmsg.mag_field.magnetic_field.y=float(fields[5])
        vnmsg.mag_field.magnetic_field.z=float(fields[6])

        vnmsg.imu.linear_acceleration.x=float(fields[7])
        vnmsg.imu.linear_acceleration.y=float(fields[8])
        vnmsg.imu.linear_acceleration.z=float(fields[9])

        
        vnmsg.imu.angular_velocity.x=float(fields[10])
        vnmsg.imu.angular_velocity.y=float(fields[11])
        vnmsg.imu.angular_velocity.z=float((fields[12]).split('*')[0])
       

        vnmsg.imu_read=str(VNYMR_string)
       
       
    return vnmsg ,title






def callback(data):
   d=data.data
   print('hi',d)
   imu_dat =d
   if imu_dat == ' ':
     rospy.logwarn("No data")
   else:                      
      try: 
            x=time.time()
            y=(int(time.time()))
            z=x-y
            [vnmsg,title]=VNYMR(imu_dat,x,z)
                    
                   
            if title == "$VNYMR":
                IMU_pub.publish(vnmsg)
                rospy.loginfo("Published message: %s", vnmsg)
      except: 
                 rospy.logwarn("Data exception: "+imu_dat)
           





if __name__ == '__main__':
    bag_path = '/home/sri/imu/src/data/LocationC.bag'
    SENSOR_NAME = "IMU"
    vnmsg = Vectornav()
    
    rospy.init_node("bag_driver", anonymous=True)
    bag_pub = rospy.Publisher("bag", String, queue_size=10)
    rospy.Subscriber("bag", String, callback)
    IMU_pub = rospy.Publisher("/imu", Vectornav, queue_size=10)
    
    vnmsg = Vectornav()
 

    rospy.sleep(0.2)  
    


    rate = rospy.Rate(100)
    
    rospy.logdebug("Initialization complete")
    
    rospy.loginfo("Publishing IMU")
    
   

   

    try:
        bag = rosbag.Bag(bag_path, 'r')
        
        try:
            for topic, msg, t in bag.read_messages(topics=['/vectornav']):
                bag_pub.publish(msg.data)
                print(msg.data)
                rate.sleep()
                
        except rospy.ROSInterruptException:
            print('bye')

    except rosbag.bag.ROSBagException as e:
        print(f"Error opening or reading bag file: {e}")

    finally:
        bag.close()

            
            




  

 
