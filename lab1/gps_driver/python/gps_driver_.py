#!/usr/bin/env python3

import rospy
import rosbag
import serial
from std_msgs.msg import String 
from datetime import datetime
import utm
import time
from gps_driver.msg import Customgps

serialPortAddr = '/dev/pts/5'
bag_filename = "gnss_data.bag"
bag = rosbag.Bag(bag_filename, 'w')
    

def GPGGA(gpgga_string):

    fields = gpgga_string.split(',')
    title=fields[0]
    

    if title == "b'$GPGGA":
        cmsg.UTC = float(fields[1])
        cmsg.latitude = float(fields[2])
        cmsg.longitude = float(fields[4])
        cmsg.LatitudeDir = fields[3]
        cmsg.LongitudeDir = fields[5]
        cmsg.altitude= float(fields[9])
        cmsg.latitude = float(fields[2])
        cmsg.gpgga_read=gpgga_string
        
        try:
            cmsg.hdop = float(fields[8])
        except ValueError:
                  cmsg.hdop =0
                  

    [CurrentTime,CurrentTimeNsec]=UTC_to_Epoc(cmsg.UTC)
    cmsg.header.frame_id = 'GPS1_Frame'
    cmsg.header.stamp.secs =int(CurrentTime)
    cmsg.header.stamp.nsecs=int(CurrentTimeNsec)
    
    
    
        
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

    cdt = datetime.now()
    bod = cdt.replace(hour=0, minute=0, second=0, microsecond=0)
    TimeSinceEpochBOD= int(bod.timestamp())

    CurrentTime = TimeSinceEpochBOD + int(UTC)
    UTC_sec= UTC-int(UTC)
    CurrentTimeNsec = int( UTC_sec * 1e9)
    
    return [CurrentTime, CurrentTimeNsec]
           



def Serial_read(serialPortAddr,serial_baud,sampling_rate,timeout=1):
    
    serialPort = serial.Serial(serialPortAddr,serial_baud,sampling_rate,timeout=1)
    
    gpgga_read = str(serialPort.readline())

    if "GPGGA" in gpgga_read:

        cmsg.gpgga_read= gpgga_read
        serialPort.close()

    return cmsg.gpgga_read



if __name__ == '__main__':
    cmsg = Customgps()
    rospy.init_node("GNSS", anonymous=True)
    serial_baud = rospy.get_param('~baudrate', 4800)
    sampling_rate = rospy.get_param('~sampling_rate', 5.0)
    GPS_pub = rospy.Publisher("chatter", Customgps, queue_size=10)

    

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing GNSS")

try:
    while not rospy.is_shutdown():
        cmsg = Customgps()
        cmsg.gpgga_read = Serial_read(serialPortAddr, serial_baud, sampling_rate,timeout=1)

        if cmsg.gpgga_read == ' ':
            rospy.logwarn(" No data")
            continue

        try:
            [cmsg, title] = GPGGA(cmsg.gpgga_read)
            Lat = Min_to_DegDec(cmsg.latitude)
            Long = Min_to_DegDec(cmsg.longitude)
            LatitudeSigned = LatLongSignConvetion(Lat, cmsg.LatitudeDir)
            cmsg.latitude = Lat
            LongitudeSigned = LatLongSignConvetion(Long, cmsg.LongitudeDir)
            cmsg.longitude = Long
            [cmsg.utm_easting, cmsg.utm_northing, cmsg.zone, cmsg.letter] = lat_long_to_utm(LatitudeSigned, LongitudeSigned)
            print(title)
            if title == "b'$GPGGA":
                GPS_pub.publish(cmsg)
                rospy.loginfo("Published message: %s", cmsg)
                bag.write("chatter", cmsg)
               

        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down GPS node...")
            
finally:
     bag.close() 
