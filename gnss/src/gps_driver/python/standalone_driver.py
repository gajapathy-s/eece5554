#!/usr/bin/env python3

import rospy
import rosbag
import serial
from datetime import datetime
import utm

from gps_driver.msg import Customgps

bag_filename="gps_open_still"


def GPGGA(gpgga_string):

    fields = gpgga_string.split(',')
    title=str(fields[0])
    

    if title == "$GPGGA":
        cmsg.UTC = float(fields[1])
        cmsg.latitude = float(fields[2])
        cmsg.longitude = float(fields[4])
        cmsg.LatitudeDir = fields[3]
        cmsg.LongitudeDir = fields[5]
        cmsg.altitude= float(fields[9])
        cmsg.latitude = float(fields[2])
        cmsg.gpgga_read=gpgga_string
        cmsg.hdop = float(fields[8])
                  
                  

    [CurrentTime,CurrentTimeNsec]=UTC_to_Epoc(cmsg.UTC)
    cmsg.header.frame_id = 'GPS1_Frame'
    cmsg.header.stamp.secs =int(CurrentTime)
    cmsg.header.stamp.nsecs=int(CurrentTimeNsec)
    # cmsg.header.stamp=rospy.Time.now()
    
 
 
 
 
 
    return cmsg ,title
   
 
def Min_to_DegDec(lat_long):
    deg = float(lat_long) // 100
    mins = float(lat_long) - (deg * 100)
    degDec = deg + (mins / 60)
    return degDec



def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir == "W" or LatOrLongDir == "S":
         LatOrLong = (-1) * float(LatOrLong)
    elif LatOrLongDir == "E" or LatOrLongDir == "N":
         LatOrLong = float(LatOrLong)
    elif LatOrLongDir == " ":
         rospy.logwarn(" No data")
    return LatOrLong




def lat_long_to_utm(Lat, Long):
    UTM = utm.from_latlon(Lat, Long)
    cmsg.utm_easting= UTM[0]
    cmsg.utm_northing= UTM[1]
    cmsg.zone = UTM[2]
    cmsg.letter= UTM[3]
    return cmsg.utm_easting, cmsg.utm_northing, cmsg.zone, cmsg.letter

   
   
def UTC_to_Epoc(UTC):

    hours = int(UTC // 10000)
    minutes = int((UTC % 10000) // 100)
    seconds = UTC % 100
    UTCinSecs = hours * 3600 + minutes * 60 + seconds
    
    currentTime = time.gmtime()

    TimeSinceEpochBOD = time.mktime(currentTime[:3] + (0, 0, 0) + currentTime[6:])
 
    CurrentTime = TimeSinceEpochBOD + UTCinSecs - time.timezone 

    UTC_sec= UTC-int(UTC)
    
    CurrentTimeNsec = int( round((UTC_sec),2) * 1e9)
    
    return [CurrentTime, CurrentTimeNsec]
           


if __name__ == '__main__':
    
    SENSOR_NAME = "GPS"
    
    cmsg = Customgps()
    rospy.init_node("Standalone_driver", anonymous=True)
    
    
    serial_port = rospy.get_param('~port','/dev/pts/5')
    serial_baud = rospy.get_param('~baudrate',4800)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
    #bag_filename = rospy.get_param('~bag_name', 'gpsbag')
    port = serial.Serial(serial_port, serial_baud, timeout=1)
    
    
    rospy.logdebug("Using USB sensor on port "+serial_port+" at "+str(serial_baud))
  
    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
    
    rospy.sleep(0.2)  
    
    GPS_pub = rospy.Publisher("/gps", Customgps, queue_size=10)
    rate = rospy.Rate(10)
    bag = rosbag.Bag(bag_filename, 'w')
    
    rospy.logdebug("Initialization complete")
    
    rospy.loginfo("Publishing GPS")
    
    sleep_time = 1/sampling_rate - 0.025
try:
    try:
        while not rospy.is_shutdown():
            cmsg = Customgps()
            gpgga_read =(port.readline().decode('utf-8').strip())
            cmsg.gpgga_read= str(gpgga_read)
           # print((cmsg.gpgga_read))

            if cmsg.gpgga_read == ' ':
                rospy.logwarn("No data")
            else:
                                      
                    try: 
                     [cmsg, title] = GPGGA(cmsg.gpgga_read)
                     Lat = Min_to_DegDec(cmsg.latitude)
                     Long = Min_to_DegDec(cmsg.longitude)
                     LatitudeSigned = LatLongSignConvetion(Lat, cmsg.LatitudeDir)
                     cmsg.latitude = LatitudeSigned
                     LongitudeSigned = LatLongSignConvetion(Long, cmsg.LongitudeDir)
                     cmsg.longitude = LongitudeSigned
                     [cmsg.utm_easting, cmsg.utm_northing, cmsg.zone, cmsg.letter] = lat_long_to_utm(LatitudeSigned, LongitudeSigned)
                     #print(title)
                     if title == "$GPGGA":
                            GPS_pub.publish(cmsg)
                            rospy.loginfo("Published message: %s", cmsg)
                            bag.write("/gps", cmsg)
                           
                            rospy.sleep(sleep_time)
                
                    except: 
                        rospy.logwarn("Data exception: "+gpgga_read)
                        continue        
    except rospy.ROSInterruptException:
            port.close()
             
        
    except serial.serialutil.SerialException:
     rospy.loginfo("Shutting down GPS node...")
       
finally:
     bag.close() 
            
            
    
    

        
        
