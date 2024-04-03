#!/usr/bin/env python3


import rosbag
import time
import rospy
import serial
import math
from dead_recon.msg import Vectornav



def vn_rate(port):
     if port.isOpen=="False":
          port.Open()
     port.write(b'$VNWRG,35,1,0,0,0*XX\r\n')
     time.sleep(1)
     port.write(b'$VNWRG.07,40*XX\r\n')
     time.sleep(1)
     port.write(b'$VNWRG.06,14*XX\r\n')
     time.sleep(1)
     print("Rate = 40 kHz")
     port.close()

#def convert_to_quarternions(Y,P,R):
  
     #return vnmsg
 





def convert_to_quaternion(VNYMR_string,x,z):
    
    fields = VNYMR_string.split(',')
    title=str(fields[0])
    
    vnmsg.header.frame_id = 'imu1_frame'
    vnmsg.header.stamp.secs=x
    vnmsg.header.stamp.nsecs=z
    
    vnmsg.imu.header.frame_id = 'imu1_frame'
    vnmsg.imu.header.stamp.secs=x
    vnmsg.imu.header.stamp.nsecs=z



    vnmsg.mag_field.header.frame_id = 'imu1_frame'
    vnmsg.mag_field.header.stamp.secs=x
    vnmsg.mag_field.header.stamp.nsecs=z
    
    
    if title == "$VNYMR":

        Y=float(fields[1])
        P= float(fields[2])
        R=float(fields[3])

        
        yaw=Y
        pitch=P
        roll=R

        h_y=yaw
        h_p=pitch
        h_r=roll


        DCM = [
        [math.cos(math.radians(h_p)) * math.cos(math.radians(h_y)), math.cos(math.radians(h_p)) * math.sin(math.radians(h_y)), -math.sin(math.radians(h_p))],
        [-math.sin(math.radians(h_r)) * math.sin(math.radians(h_p)) * math.cos(math.radians(h_y)) + math.cos(math.radians(h_r)) * math.sin(math.radians(h_y)),
        math.cos(math.radians(h_r)) * math.cos(math.radians(h_y)) + math.sin(math.radians(h_r)) * math.sin(math.radians(h_p)) * math.sin(math.radians(h_y)),
        math.cos(math.radians(h_p)) * math.sin(math.radians(h_r))],
        [math.cos(math.radians(h_r)) * math.sin(math.radians(h_p)) * math.cos(math.radians(h_y)) + math.sin(math.radians(h_r)) * math.sin(math.radians(h_y)),
        -math.cos(math.radians(h_r)) * math.sin(math.radians(h_y)) + math.sin(math.radians(h_r)) * math.sin(math.radians(h_p)) * math.cos(math.radians(h_y)),
        math.cos(math.radians(h_p)) * math.cos(math.radians(h_r))]
    ]
      
      
      

        vnmsg.imu.orientation.x = 0.25 * math.sqrt(1 + DCM[0][0] - DCM[1][1] - DCM[2][2])
        vnmsg.imu.orientation.y = 0.25 * math.sqrt(1 - DCM[0][0] + DCM[1][1] - DCM[2][2])
        vnmsg.imu.orientation.z = 0.25 * math.sqrt(1 - DCM[0][0] - DCM[1][1] + DCM[2][2])
        vnmsg.imu.orientation.w = 0.25 * math.sqrt(1 + DCM[0][0] + DCM[1][1] + DCM[2][2])

        #vnmsg.imu.orientation.w = float((math.cos(h_y) * math.cos(h_p) * math.cos(h_r)) + (math.sin(h_y) * math.sin(h_p) * math.sin(h_r)))
        #vnmsg.imu.orientation.x = float((math.cos(h_y) * math.sin(h_p) * math.cos(h_r)) - (math.sin(h_y) * math.cos(h_p) * math.sin(h_r)))
        #vnmsg.imu.orientation.y =float((math.cos(h_y) * math.cos(h_p) * math.sin(h_r)) + (math.sin(h_y) * math.sin(h_p) * math.cos(h_r)))
        #vnmsg.imu.orientation.z = float((math.sin(h_y) * math.sin(h_p) * math.cos(h_r)) - (math.cos(h_y) * math.sin(h_p) * math.sin(h_r)))



        vnmsg.mag_field.magnetic_field.x=float(fields[4])*0.0001
        vnmsg.mag_field.magnetic_field.y=float(fields[5])*0.0001
        vnmsg.mag_field.magnetic_field.z=float(fields[6])*0.0001

        vnmsg.imu.linear_acceleration.x=float(fields[7])
        vnmsg.imu.linear_acceleration.y=float(fields[8])
        vnmsg.imu.linear_acceleration.z=float(fields[9])

        
        vnmsg.imu.angular_velocity.x=float(fields[10])
        vnmsg.imu.angular_velocity.y=float(fields[11])
        vnmsg.imu.angular_velocity.z=float((fields[12]).split('*')[0])
       

        vnmsg.imu_read=str(VNYMR_string)
       
       
    return vnmsg ,title







if __name__ == '__main__':
    
    
    SENSOR_NAME = "IMU"
    vnmsg = Vectornav()
 
    rospy.init_node("imu_driver", anonymous=True)
    
    
    serial_port = rospy.get_param('~port2','/dev/ttyUSB1')
    serial_baud = rospy.get_param('~baudrate',115200)
    
    #sampling_rate = rospy.get_param('~sampling_rate',5.0)
   
    port = serial.Serial(serial_port, serial_baud, timeout=1)
    vn_rate(port)
    
    rospy.logdebug("Using USB sensor on port "+serial_port+" at "+str(serial_baud))
  
    rospy.logdebug("Initializing sensor with *$VNWRG.06,14*XX\r\n ...")
    
    
   # rospy.sleep(0.2)  
    
    IMU_pub = rospy.Publisher("/imu", Vectornav, queue_size=10)
    bag = rosbag.Bag('/home/sri/imu/imu_square.bag', 'w')
    rate = rospy.Rate(100)

   
    
    rospy.logdebug("Initialization complete")
    
    rospy.loginfo("Publishing IMU")
    
   # sleep_time = 1/sampling_rate - 0.025
   
try:
    try:
        while not rospy.is_shutdown():
            if port.isOpen() == False:
                port.open()

            imu_dat = (port.readline().decode('utf-8').strip())

            if imu_dat == ' ':
                rospy.logwarn("No data")
            else:
                try:
                    current_time = rospy.Time.now()

                    seconds = current_time.secs
                    nanoseconds = current_time.nsecs

                    x = int(seconds)
                    y = int(nanoseconds)
                    z = x - y
                    [vnmsg, title] = convert_to_quaternion(imu_dat, x, z)

                    if title == "$VNYMR":
                        IMU_pub.publish(vnmsg)
                        rospy.loginfo("Published message: %s", vnmsg)
                        bag.write("/imu", vnmsg)
                        # rospy.sleep(sleep_time)

                except Exception as e:
                    rospy.logerr("Exception processing data: %s", str(e))
                    continue

    except rospy.ROSInterruptException:
        port.close()

except serial.serialutil.SerialException:
    rospy.loginfo("Shutting down imu...")

finally:
    bag.close()
           
            
    
    

        
        