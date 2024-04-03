#!/usr/bin/env python3

import rospy
import rosbag
import sys
from bagpy import bagreader
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares 
from scipy.signal import butter, sosfilt, freqz
from scipy import integrate
import os

class nuance_correction:
 
    def __init__(self):

        self.path = '/home/sri/nuance_ws/src/data/data_driving.bag'
        self.imu_bag(self.path)
        self.gps_bag(self.path)
    
    def imu_bag(self,path):
        try:
            bag = rosbag.Bag(path)
            self.q_x = []
            self.q_y= []
            self.q_z = []
            self.q_w=[]

            self.magnetic_field_x=[]
            self.magnetic_field_y=[]
            self.magnetic_field_z=[]

            self.linear_acceleration_x=[]
            self.linear_acceleration_y=[]
            self.linear_acceleration_z=[]

            self.linear_acceleration_x_corr=[]
            self.linear_acceleration_y_corr=[]

            self.angular_velocity_x=[]
            self.angular_velocity_y=[]
            self.angular_velocity_z=[]

            self.ang_vel=[]

            self.ns=[]
            self.time=[]
             

            
            for topic, msg, t in bag.read_messages(topics=['/imu']):
                

                #print(msg.mag_field.magnetic_field.z)
                
                self.time.append(msg.header.stamp.secs)
                self.ns.append((msg.header.stamp.nsecs)*(1e-9))

                self.q_x.append(msg.imu.orientation.x)
                self.q_y.append(msg.imu.orientation.y)
                self.q_z.append(msg.imu.orientation.z)
                self.q_w.append(msg.imu.orientation.w)


                self.magnetic_field_x.append(msg.mag_field.magnetic_field.x)
                self.magnetic_field_y.append(msg.mag_field.magnetic_field.y)
                self.magnetic_field_z.append(msg.mag_field.magnetic_field.z)

                self.linear_acceleration_x.append(msg.imu.linear_acceleration.x)
                self.linear_acceleration_y.append(msg.imu.linear_acceleration.y)
                self.linear_acceleration_z.append(msg.imu.linear_acceleration.z)

                self.angular_velocity_x.append(msg.imu.angular_velocity.x)
                self.angular_velocity_y.append(msg.imu.angular_velocity.y)
                self.angular_velocity_z.append(msg.imu.angular_velocity.z)

                

        except rosbag.bag.ROSBagException as e:
            print(f"Error opening or reading bag file: {e}")

        finally:
            bag.close()
          
            self.q_x=np.array(self.q_x)
            self.q_y=np.array(self.q_y)
            self.q_z=np.array(self.q_z)
            self.q_w=np.array(self.q_w)

            self.q_yaw_angle_imu = np.arctan2(2*(self.q_w*self.q_x + self.q_y*self.q_z), 1 - 2*(np.square(self.q_x) + np.square(self.q_y)))
            
            
            self.magnetic_field_x = np.array(self.magnetic_field_x)
            self.magnetic_field_y = np.array(self.magnetic_field_y)
            self.magnetic_field_z = np.array(self.magnetic_field_z)

            self.angular_velocity_x = np.array(self.angular_velocity_x)
            self.angular_velocity_y = np.array(self.angular_velocity_y)
            self.angular_velocity_z = np.array(self.angular_velocity_z)

            self.angular_velocity_z=(self.angular_velocity_z*(180/math.pi))*(180/math.pi)
         
            self.linear_acceleration_x = np.array(self.linear_acceleration_x)
            self.linear_acceleration_y = np.array(self.linear_acceleration_y)
            self.linear_acceleration_z = np.array(self.linear_acceleration_z)
            
            
            self.time_elapsed_sec = np.array(self.time)
            self.time_elapsed_ns = np.array(self.ns)
            
            self.time_elapsed = self.time_elapsed_sec + self.time_elapsed_ns
            self.time_elapsed = self.time_elapsed - self.time_elapsed[0]

    def gps_bag(self,path):
        try:
            bag = rosbag.Bag(path)
            self.latitude = []
            self.longitude = []
            self.altitude = []

            self.utm_easting=[]
            self.utm_northing=[]
            self.LatitudeDir=[]
            self.LongitudeDir=[]
            self.UTC=[]

            self.q_zone=[]
            self.letter=[]
            self.hdop=[]
           
            self.UE=[]
            self.UN=[]

            self.ns_gps=[]
            self.time_gps=[]

            for topic, msg, t in bag.read_messages(topics=['/gps']):
                dat = msg

                
                
                self.time_gps.append(msg.header.stamp.secs)
                self.ns_gps.append((msg.header.stamp.nsecs)*(1e-9))

                self.latitude.append(msg.latitude)
                self.longitude.append(msg.longitude)
                self.altitude.append(msg.altitude)

                

                self.utm_easting.append(msg.utm_easting)
                self.utm_northing.append(msg.utm_northing)
                
               


               # self.LatitudeDir.append(msg.LatitudeDir)
               # self.LongitudeDir.append(msg.LongitudeDir)

               # self.UTC.append(msg.UTC)

                self.q_zone.append(msg.zone)
                self.letter.append(msg.letter)
                self.hdop.append(msg.hdop)
        

        except rosbag.bag.ROSBagException as e:
            print(f"Error opening or reading bag file: {e}")

        finally:
            
            bag.close()
            
            self.latitude = np.array(self.latitude)
            self.longitude = np.array(self.longitude)
            self.altitude = np.array(self.altitude)

            self.utm_easting = np.array(self.utm_easting)
            self.utm_northing= np.array(self.utm_northing)


            self.LatitudeDir = np.array(self.LatitudeDir)
            self.LongitudeDir= np.array(self.LongitudeDir)

            self.UTC = np.array(self.UTC)
            self.q_zone= np.array(self.q_zone)


            self.letter = np.array(self.letter)
            self.hdop= np.array(self.hdop)
            
            self.time_gps=np.array(self.time_gps)
            time_elapsed_sec_gps = np.array(self.time_gps)
            time_elapsed_ns_gps = np.array(self.ns_gps)

            
                
            self.time_elapsed_gps = time_elapsed_sec_gps + time_elapsed_ns_gps
            self.time_elapsed_gps = self.time_elapsed_gps - self.time_elapsed_gps[0]
 
    def num_int_trap(self, f_x, t):
        time = t
        func = f_x
        integrated = integrate.cumulative_trapezoid(func, time, initial=0)
        return integrated

    def butter_filter(self,raw_data, cutoff_freq, sampl_freq, filt_type, filt_order):
                nyq_freq = sampl_freq / 2 #set the Nyquist frequency (important to avoid aliasing)
                sos = butter(N = filt_order, Wn = cutoff_freq / nyq_freq, btype=filt_type, analog=False, output='sos')
                filtered_data = sosfilt(sos, raw_data)
                return sos, filtered_data
               
    def l_h_filter(self,data,type,cf):
                # Setting filter requirements
                order = 3 #you can increase this to make the filter "sharper"
                sampl_freq = 40 #change to sampling frequency of your data collection
                cutoff_freq = cf #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 

                filter_type=type
                

                # Filtering and plotting
                [sos, y] = self.butter_filter(data, cutoff_freq, sampl_freq, filter_type, order)

               
                return y 

    def magnetometer_yaw(self):

            # The magnetometer yaw estimation before and after hard and soft iron calibration vs. time
                

                fig, ax = plt.subplots()


                self.heading_raw=np.arctan2(self.raw_mag[0],self.raw_mag[1])
                self.heading_corr=np.arctan2(self.calibrated_mag[0],self.calibrated_mag[1])
                print(len(self.heading_corr))
                print(len(self.time_elapsed))


                ax.plot(self.time_elapsed,self.heading_raw*(180/math.pi),label="Raw data Yaw angle")
                ax.plot(self.time_elapsed,-(self.heading_corr)*(180/math.pi), label="calibrated data Yaw angle")
                ax.axis('equal')
                ax.legend()
                plt.xlabel('Time elapsed (sec)')
                plt.ylabel('Magnetometer Yaw angle (degrees)')
                plt.title(' Plot Comparing Raw magnetometer Yaw with the Corrected Yaw angle')
                plt.show()
        
    def Gyro(self):   
        #Plot of gyro yaw estimation vs. time

      
        
        plt.plot(self.time_elapsed,(self.angular_velocity_z)) 
        plt.axis('equal')
        plt.legend()
        plt.xlabel('Time (sec)')
        plt.ylabel('Z component of Angular velocity (degrees/sec)')
       
        plt.title(' Gyro yaw estimation vs. time')
        plt.show()

    def filter(self):
         #Low pass filter of magnetometer data, high pass filter of gyro data, complementary filter output, and IMU heading estimate as 4 subplots on one plot
        self.filtered_mag_data=[]
        self.filtered_gyro_yaw_data=[]
        self.fused=[]

        fig, axs = plt.subplots(2, 2)

        a = 0.5

        self.filtered_mag_data =-(self.l_h_filter(self.heading_corr, 'lowpass',0.1))
       
        axs[0, 0].plot(self.time_elapsed, -(self.heading_corr*(180/math.pi)), "b-", label="data")
        axs[0, 0].plot(self.time_elapsed, self.filtered_mag_data*(180/math.pi), "g-", linewidth=2, label="filtered data")
        axs[0, 0].set_xlabel("Time [sec]")  
        axs[0, 0].set_ylabel('Yaw angle (degrees)')
        axs[0, 0].set_title("Low pass filter data")  
        axs[0, 0].grid()
        axs[0, 0].legend()
        plt.subplots_adjust(hspace=0.35)



        filtered_gyro_z_data = self.l_h_filter(self.angular_velocity_z, 'highpass',0.000001)

        self.filtered_gyro_yaw_data = -(self.num_int_trap(filtered_gyro_z_data, self.time_elapsed))

        


        self.Yaw_angle = self.num_int_trap(self.angular_velocity_z, self.time_elapsed)
       
        axs[0, 1].plot(self.time_elapsed, -(self.Yaw_angle), "b-", label="data")
        axs[0, 1].plot(self.time_elapsed,(self.filtered_gyro_yaw_data), "g-", linewidth=2, label="filtered data")
        axs[0, 1].set_xlabel("Time [sec]")  
        axs[0, 1].set_ylabel('Yaw angle (degrees)')
        axs[0, 1].set_title("High pass filter data")  
        axs[0, 1].grid()
        axs[0, 1].legend()

        self.fused = (a * (self.filtered_mag_data*(180/math.pi))) + ((1 - a) * (self.filtered_gyro_yaw_data))

        axs[1, 0].plot(self.time_elapsed, self.fused, label="Complementary filter")
        axs[1, 0].axis('equal')
        axs[1, 0].legend()
        axs[1, 0].set_xlabel('Time elapsed (sec)')
        axs[1, 0].set_ylabel('Yaw angle (degrees)')
        axs[1, 0].set_title('Complementary filter to combine the yaw measurements')

        axs[1, 1].plot(self.time_elapsed,-((( self.q_yaw_angle_imu)*(180/math.pi))*(180/math.pi)), color='m')
        axs[1, 1].set_xlabel('Time elapsed (sec)')
        axs[1, 1].set_ylabel('Yaw angle (degrees)')
        axs[1, 1].set_title('IMU heading estimate')

        # Adjust layout to prevent overlap of titles
        plt.tight_layout()
        plt.show()


        fig, ax = plt.subplots()
        ax.plot(self.time_elapsed, -(self.heading_corr*(180/math.pi)), "b-", label="Magnetometer_heading")
        ax.plot(self.time_elapsed, self.filtered_mag_data*(180/math.pi), "g-", linewidth=2, label="Magnetometer_heading filtered data")
        ax.plot(self.time_elapsed, -(self.Yaw_angle), "b-", label="Yaw angle from Gyroscope")
        ax.plot(self.time_elapsed,(self.filtered_gyro_yaw_data), "g-", linewidth=2, label="filtered Yaw angle from Gyroscope")
        ax.plot(self.time_elapsed, self.fused, label="Complementary filter Yaw angle ")
        ax.plot(self.time_elapsed,-( (self.q_yaw_angle_imu)*(180/math.pi))*(180/math.pi), color='m',label="Yaw angle from Imu")
        ax.legend()
        ax.set_xlabel('Time elapsed (sec)')
        ax.set_ylabel('Yaw angle (degrees)')
        ax.set_title(' Yaw angle estimate from various sources')
        plt.show()

    def vel(self):
        
     

        self.gps_vel = np.zeros(len(self.time_elapsed_gps) - 1)

        self.UE=self.utm_easting-np.mean(self.utm_easting)
        
        self.UE=self.utm_easting-self.utm_easting[0]

        self.UN=self.utm_northing-np.mean(self.utm_northing)

        self.UN=self.utm_northing-self.utm_northing[0]

       

        for i in range(len(self.time_elapsed_gps) - 1):
            self.gps_vel[i] = (np.sqrt((np.square(self.UN[i + 1] - self.UN[i])) +(np.square(self.UE[i + 1] - self.UE[i])))) / (self.time_elapsed_gps[i + 1] - self.time_elapsed_gps[i])

        
        self.linear_acceleration_x = self.linear_acceleration_x - np.mean(self.linear_acceleration_x)
        self.velocity_linear_x = self.num_int_trap(self.linear_acceleration_x, self.time_elapsed)

    


        unique_velocity_linear_x = []
        for i in self.time_gps:
            indices = np.where(self.time_elapsed_sec == i)
            unique_velocity_linear_x.append(self.velocity_linear_x[indices[0][0]])

        unique_velocity_linear_x = np.array(unique_velocity_linear_x)
        dt_up_copy = np.copy(self.time_gps)
        self.time_gps -= self.time_gps.min()

        m,b = np.polyfit(self.time_gps,unique_velocity_linear_x,1)
        dist_lobf = np.absolute(self.time_gps[:]*m - unique_velocity_linear_x[:] + b)/(np.sqrt(m**2+1))
    
        



        fig, ax = plt.subplots()
        ax.plot(self.time_elapsed_gps, dist_lobf, "g", label="Imu Velocity aftet correction ")
        ax.plot(self.time_elapsed,self.velocity_linear_x , "b",label="Imu Velocity before correction")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Linear Velocity (m/s)")
        plt.title("Velocity estimation  correction using acceleration from imu")
        plt.legend()
        plt.show()
        
        plt.plot(self.time_elapsed_gps, dist_lobf, "g", label="Imu Velocity ")
        plt.plot(self.time_elapsed_gps[:-1],self.gps_vel , "b",label="GPS Velocity")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Linear Velocity (m/s)")
        plt.title("Velocity estimation using acceleration from imu")
        plt.legend()
        plt.show()

    
        gps_time_comp_yaw = []
        ang_vel_z=[]
        for i in dt_up_copy:
            indices = np.where(self.time_elapsed_sec == i)
            gps_time_comp_yaw.append(self.fused[indices[0][0]])
            ang_vel_z.append(self.angular_velocity_z[indices[0][0]])
                 

        gps_time_comp_yaw = np.array(gps_time_comp_yaw)

    


        self.omg_x_dash=ang_vel_z*(dist_lobf)

        fig, ax = plt.subplots()
        ax.plot(self.time_elapsed,  self.linear_acceleration_y,label="Observed Acceleration")
        ax.plot(self.time_elapsed_gps,  self.omg_x_dash*((np.pi)/180),label=" Acceleration Correction")
        ax.legend()
        plt.xlabel('Time Elapsed')
        plt.ylabel('Observed acceleration (m/s^2)')
        plt.title('Acceleration over Time')
        plt.show()
        
        
        

        self.Vel_easting =  self.velocity_linear_x*np.cos(self.heading_corr)
        self.Vel_northing =  self.velocity_linear_x*np.sin(self.heading_corr)

        #self.Vel_easting =(dist_lobf * np.cos(gps_time_comp_yaw) )
        #self.Vel_northing =(dist_lobf * np.sin(gps_time_comp_yaw))
        
        self.taj_x= self.num_int_trap(self.Vel_easting, self.time_elapsed)
        self.taj_y=self.num_int_trap(self.Vel_northing, self.time_elapsed)

        
        
                

        fig, ax = plt.subplots()       
        ax.scatter(-self.taj_x[0:11500], self.taj_y[0:11500], s=2, label='IMU Trajectory', color='blue')

        
        ax.plot(self.UE, self.UN, label="GPS Trajectory", color='red')
        ax.set_xlabel('Easting distance (meters)')
        ax.set_ylabel('Northing distance (meters)')
        plt.legend()
        plt.title("GPS Trajectory & imu Trajectory")
        plt.show()
  
    def YAW(self):
        fig, ax = plt.subplots()

        self.q_Yaw_angle = self.num_int_trap(self.angular_velocity_z, self.time_elapsed)

        

        ax.plot(self.time_elapsed,self.filtered_gyro_yaw_data,label=" Yaw rate from gyro sensor Integrated to get yaw angle")
        ax.plot(self.time_elapsed,-(self.heading_corr*(180/math.pi)), label="calibrated data Yaw angle")
        ax.axis('equal')
        ax.legend()
        plt.xlabel('Time elapsed (sec)')
        plt.ylabel('Yaw angle (degrees)')
        plt.title(' Plot Comparing Magnetometer Yaw vs. Yaw Integrated from Gyroscope')
        plt.show()
        



 
        
        ax.axis('equal')
       # ax.legend()
       # plt.xlabel('Time elapsed (sec)')
       # plt.ylabel('Yaw angle (degrees)')
       # plt.title(' Plot Comparing Magnetometer Yaw vs. Yaw Integrated from Gyroscope')
        plt.show()

    def aux_plots(self):


        '''
        fig, (plot1, plot2, plot3) = plt.subplots(1, 3)

        plot1.plot(self.time_elapsed, self.angular_velocity_x*(180/math.pi), label="angular velocity x")
        plot1.set_xlabel('Time (sec)')
        plot1.set_ylabel('X angular velocity (degrees/sec)')

        plot2.plot(self.time_elapsed, self.angular_velocity_y*(180/math.pi), label="angular velocity y")
        plot2.set_xlabel('Time (sec)')
        plot2.set_ylabel('Y angular velocity (degrees/sec)')

        plot3.plot(self.time_elapsed, self.angular_velocity_z*(180/math.pi), label="angular velocity z")
        plot3.set_xlabel('Time (sec)')
        plot3.set_ylabel('Z angular velocity (degrees/sec)')
        fig.suptitle("Angular Velocities vs Time", fontsize=16)

        fig, (plot3, plot4, plot5) = plt.subplots(1, 3)

        integrated = self.num_int_trap(self.angular_velocity_x*(180/math.pi), self.time_elapsed)
        plot3.plot(self.time_elapsed, integrated, label="angle x")
        plot3.set_xlabel('Time (sec)')
        plot3.set_ylabel('angle X (degrees)')

        integrated = self.num_int_trap(self.angular_velocity_y*(180/math.pi), self.time_elapsed)
        plot4.plot(self.time_elapsed, integrated, label="angle y")
        plot4.set_xlabel('Time (sec)')
        plot4.set_ylabel('angle Y (degrees)')

        integrated = self.num_int_trap(self.angular_velocity_z*(180/math.pi), self.time_elapsed)
        plot5.plot(self.time_elapsed, integrated, label="angle z")
        plot5.set_xlabel('Time (sec)')
        plot5.set_ylabel('angle Z (degrees)')
        fig.suptitle("Angle X,Y,Z vs Time", fontsize=16)

        fig, plot6 = plt.subplots(1, 1)
        heading = np.arctan2(self.magnetic_field_y, self.magnetic_field_x)
        plot6.scatter(self.time, heading, label="absolute heading vs time") 
        plot6.set_xlabel('Time (secs)')
        plot6.set_ylabel('Heading')
        fig.suptitle("Absolute heading vs time", fontsize=16)
        
        fig, ((plot7, plot8, plot9), (plot10, plot11, plot12), (plot13, plot14, plot15)) = plt.subplots(3, 3)

        plot7.plot(self.time_elapsed, self.linear_acceleration_x, label="linear acceleration x")
        plot7.set_xlabel('Time (sec)')
        plot7.set_ylabel('linear acceleration x (m/s^2)')

        plot8.plot(self.time_elapsed, self.linear_acceleration_y_corr, label="linear acceleration y")
        plot8.set_xlabel('Time (sec)')
        plot8.set_ylabel('linear acceleration y (m/s^2)')

        plot9.plot(self.time_elapsed, self.linear_acceleration_z, label="linear acceleration z")
        plot9.set_xlabel('Time (sec)')
        plot9.set_ylabel('linear acceleration z (m/s^2)')

        integrated = self.num_int_trap(self.linear_acceleration_x, self.time_elapsed)
        plot10.plot(self.time_elapsed, integrated, label="linear velocity x")
        plot10.set_xlabel('Time (sec)')
        plot10.set_ylabel('linear velocity x (m/s)')

        integrated = self.num_int_trap(self.linear_acceleration_y_corr, self.time_elapsed)
        plot11.plot(self.time_elapsed,integrated, label="linear velocity y")
        plot11.set_xlabel('Time (sec)')
        plot11.set_ylabel('linear velocity y (m/s)')
        integrated = self.num_int_trap(self.linear_acceleration_z, self.time_elapsed)
        plot12.plot(self.time_elapsed, integrated, label="linear velocity z")
        plot12.set_xlabel('Time (sec)')
        plot12.set_ylabel('linear velocity z (m/s)')

        integrated = self.num_int_trap(self.linear_acceleration_x, self.time_elapsed)
        integrated_2 = self.num_int_trap(integrated, self.time_elapsed)
        plot13.plot(self.time_elapsed, integrated_2, label="displacement x")
        plot13.set_xlabel('Time (sec)')
        plot13.set_ylabel('displacement (m)')

        integrated = self.num_int_trap(self.linear_acceleration_y_corr, self.time_elapsed)
        integrated_2 = self.num_int_trap(integrated, self.time_elapsed)
        plot14.plot(self.time_elapsed, integrated_2, label="displacement  y")
        plot14.set_xlabel('Time (sec)')
        plot14.set_ylabel('displacement (m)')

        integrated = self.num_int_trap(self.linear_acceleration_z, self.time_elapsed)
        integrated_2 = self.num_int_trap(integrated, self.time_elapsed)
        plot15.plot(self.time_elapsed, integrated_2 ,label="displacement  z")
        plot15.set_xlabel('Time (sec)')
        plot15.set_ylabel('displacement  z (m)')

        fig.suptitle("Linear acceleration,linear velocity, displacement : X , Y , Z vs time", fontsize=16)

        plt.show()
        '''

    def heading (self,path):
        try:
                bag = rosbag.Bag(path, 'r')
                
                magnetic_field_x=[]
                magnetic_field_y=[]
                magnetic_field_z=[]

                
            

                ns=[]
                time=[]


                try:
                    for topic, msg, t in bag.read_messages(topics=['/imu']):
                        dat = msg
                        
                    # print(dat)
                        time.append(msg.header.stamp.secs)
                        ns.append((msg.header.stamp.nsecs)*(1e-9))
                                    

                        magnetic_field_x.append(msg.mag_field.magnetic_field.x)
                        magnetic_field_y.append(msg.mag_field.magnetic_field.y)
                        magnetic_field_z.append(msg.mag_field.magnetic_field.z)

                        
                
                except rospy.ROSInterruptException:
                    print('bye')

        except rosbag.bag.ROSBagException as e:
                print(f"Error opening or reading bag file: {e}")

        finally:
            
            magnetic_field_x=np.array(magnetic_field_x)
            magnetic_field_y=np.array(magnetic_field_y)
            magnetic_field_z=np.array(magnetic_field_z)

            bag.close()

        return magnetic_field_x,magnetic_field_y

    def correction(self):
            path3='/home/sri/nuance_ws/src/data/data_going_in_circles.bag'

            [magnetic_field_x,magnetic_field_y]=self.heading(path3)

            

            def distortion_model(X_meas, dist_params):
                x = dist_params[0] * (X_meas[0] - dist_params[4]) + dist_params[1]*(X_meas[1] - dist_params[5])
                y = dist_params[2] * (X_meas[0] - dist_params[4]) + dist_params[3]*(X_meas[1] - dist_params[5])
                X = np.array([x,y])
                return X
                
            #This function finds the difference between a circle and my transformed measurement
            def residual(p, X_mag, X_meas):
                return (X_mag - distortion_model(X_meas, p)).flatten()

            #Completely made up data set for "perfect" magnetometer readings
            field_strength = 20509e-9 #The horizontal magnetic field strength in Boston is approx 20,500 nT 
            angle = np.linspace(4*(np.pi), -4*(np.pi) ,4213)
            x_mag = field_strength * np.sin(angle) 
            y_mag = field_strength * np.cos(angle) 
            X_mag = np.array([x_mag, y_mag])

            #More made up data describing the distortion of the magnetic field.
            x_meas = magnetic_field_x
            y_meas = magnetic_field_y
            X_meas = np.array([x_meas, y_meas])

            #Least squares optimization to find model coefficients
            p0 = [0,0,0,0,0,0]
            lsq_min = least_squares(residual, p0, args=(X_mag, X_meas))

            print("least square fitting values are: ")
            print(lsq_min.x)


            

            [magnetic_field_x_,magnetic_field_y_]=self.heading(self.path)

            self.raw_mag= [magnetic_field_x_,magnetic_field_y_] 

            x_meas_ = magnetic_field_x_
            y_meas_ = magnetic_field_y_
            X_meas_ = np.array([x_meas_, y_meas_])


            self.calibrated_mag = distortion_model(X_meas_, lsq_min.x) 

            #Plotting ellipse and lsq
            plt.style.use("seaborn-v0_8-dark")

            fig, ax = plt.subplots()
            ax.plot(X_mag[0],X_mag[1])
            ax.scatter(self.calibrated_mag[0],self.calibrated_mag[1], label="calibrated data")
            ax.scatter(X_meas_[0],X_meas_[1], label="measured data")
            ax.axis('equal')
            ax.legend()
            plt.xlabel('X component of magnetic field (T)')
            plt.ylabel('Y component of magnetic field (T)')
            plt.title('N vs. E components of magnetic field before and after applying calibration to  dataset.')
            plt.show()
        
if __name__ == '__main__':
    nuance_controller = nuance_correction()
try:
   
    nuance_controller.correction()
    nuance_controller.magnetometer_yaw()
    nuance_controller.Gyro()
    nuance_controller.filter()
    nuance_controller.vel()
    nuance_controller.YAW()
    

   
  
except rospy.ROSInterruptException:
    pass
