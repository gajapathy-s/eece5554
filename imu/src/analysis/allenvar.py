import rospy
import rosbag
import allantools
import numpy as np
import matplotlib.pyplot as plt


bag_path = '/home/sri/imu/src/data/LocationC.bag'





def noise_param(tau,adev):

    [tauN, N] = parameters_and_plot(tau, adev, -0.5, 'Angle Random Walk')


    [tauK, K ]= parameters_and_plot(tau, adev, 0.5, 'Rate Random Walk', scf=np.sqrt(3))


    [tauB, B] = parameters_and_plot(tau, adev, 0, 'Bias Instability', scf=np.sqrt(2 * np.log(2) / np.pi))


    tau_params = [tauN, tauK, tauB]
    params = [N, K, B]
    
    dat = list(zip(tau_params, params))
    print(dat)
    plt.figure()
    plt.loglog(tau, adev, label=r'$\sigma$')
    plt.loglog(tau, params[0] * (tau ** -0.5), '--', label=r'$\sigma_N$')
    plt.loglog(tau, params[1] * np.sqrt(tau / 3), '--', label=r'$\sigma_K$')
    plt.loglog(tau, params[2] * np.ones_like(tau), '--', label=r'$\sigma_B$')
    plt.scatter(tau_params, params, marker='o')
    
    plt.title('Allan Deviation with Noise Parameters')
    plt.xlabel(r'$\tau$ (Time Intervals)')
    plt.ylabel(r'$\sigma(\tau)$ (Allan Deviation)')
    plt.legend([r'$\sigma (Allen Variance)(rad/s)$', r'$\sigma_N (Angle Random Walk)((rad/s)/\sqrt{Hz})$', r'$\sigma_K (Rate Random walk) ((rad/s)\sqrt{Hz})$', r'$\sigma_B (Bias Instability) (rad/s)$'])
    plt.grid(True)
    plt.axis('equal')
    plt.show()



def find_slope_index(logtau, logadev, slope):
    dlogadev = np.diff(logadev) / np.diff(logtau)
    i = np.argmin(np.abs(dlogadev - slope))
    return i


def find_y_intercept(logtau, logadev, slope, i):
    b = logadev[i] - slope * logtau[i]
    return b

def parameters_and_plot(tau, adev, slope, label, scf=None):
    logtau = np.log10(tau)
    logadev = np.log10(adev)

    i = find_slope_index(logtau, logadev, slope)

  
    b = find_y_intercept(logtau, logadev, slope, i)

  
    if scf:
        param = 10 ** (b - np.log10(scf))
    else:
        param = 10 ** b

    
    
   

    return tau[i], param



def VNYMR(VNYMR_string): 
    Y = 0
    P = 0
    R = 0
    m_x = 0
    m_y = 0
    m_z = 0
    l_x = 0
    l_y = 0
    l_z = 0
    a_x=0
    a_y=0
    a_z=0
    
    fields = VNYMR_string.split(',')
    title = str(fields[0])
    
    if title == "$VNYMR":
        Y = float(fields[1].strip().replace('\x00', ''))
        P = float(fields[2].strip().replace('\x00', ''))
        R = float(fields[3].strip().replace('\x00', ''))
        
        
        m_x=float(fields[4].strip().replace('\x00', ''))
        m_y=float(fields[5].strip().replace('\x00', ''))
        m_z=float(fields[6].strip().replace('\x00', ''))

        l_x=float(fields[7].strip().replace('\x00', ''))
        l_y=float(fields[8].strip().replace('\x00', ''))
        l_z=float(fields[9].strip().replace('\x00', ''))

        a_x=float(fields[10].strip().replace('\x00', ''))
        a_y=float(fields[11].strip().replace('\x00', ''))
        a_z = float(fields[12].split('*')[0].replace('\x00', '').replace('00.0', ''))


    return Y, P, R,m_x,m_y,m_z,l_x,l_y,l_z,a_x,a_y,a_z



try:
    bag = rosbag.Bag(bag_path, 'r')
    Yaw = []
    Pitch = []
    Roll = []

    magnetic_field_x=[]
    magnetic_field_y=[]
    magnetic_field_z=[]

    linear_acceleration_x=[]
    linear_acceleration_y=[]
    linear_acceleration_z=[]

    angular_velocity_x=[]
    angular_velocity_y=[]
    angular_velocity_z=[]


    try:
        for topic, msg, t in bag.read_messages(topics=['/vectornav']):
            dat = msg.data.strip() 
           # print(dat)
            [Y, P, R,m_x,m_y,m_z,l_x,l_y,l_z,a_x,a_y,a_z] = VNYMR(dat)
            
            
            Yaw.append(Y)
            Pitch.append(P)
            Roll.append(R)


            magnetic_field_x.append(m_x)
            magnetic_field_y.append(m_y)
            magnetic_field_z.append(m_z)


            linear_acceleration_x.append(l_x)
            linear_acceleration_y.append(l_y)
            linear_acceleration_z.append(l_z)


            angular_velocity_x.append(a_x)
            angular_velocity_y.append(a_y)
            angular_velocity_z.append(a_z)


            
    
    except rospy.ROSInterruptException:
        print('bye')

except rosbag.bag.ROSBagException as e:
    print(f"Error opening or reading bag file: {e}")

finally:
 angular_velocity_x=np.array(angular_velocity_x)
 angular_velocity_y=np.array(angular_velocity_y)
 angular_velocity_z=np.array(angular_velocity_z)

 linear_acceleration_x=np.array(linear_acceleration_x)
 linear_acceleration_y=np.array(linear_acceleration_y)
 linear_acceleration_z=np.array(linear_acceleration_z)
 bag.close()




(tau_out_g_x, oadev_g_x, adeverr, n)= allantools.oadev(angular_velocity_x, rate=40.0, data_type="freq", taus="all")

(tau_out_g_y, oadev_g_y, adeverr, n) = allantools.oadev(angular_velocity_y, rate=40.0, data_type="freq", taus="all")

(tau_out_g_z, oadev_g_z, adeverr, n) = allantools.oadev(angular_velocity_z, rate=40.0, data_type="freq", taus="all")


(tau_out_a_x, oadev_a_x, adeverr, n) = allantools.oadev(linear_acceleration_x, rate=40.0, data_type="freq", taus="all")
(tau_out_a_y, oadev_a_y, adeverr, n)= allantools.oadev(linear_acceleration_y, rate=40.0, data_type="freq", taus="all")
(tau_out_a_z, oadev_a_z, adeverr, n) = allantools.oadev(linear_acceleration_z, rate=40.0, data_type="freq", taus="all")





noise_param(tau_out_g_x, oadev_g_x)
noise_param(tau_out_g_y, oadev_g_y)
noise_param(tau_out_g_z, oadev_g_z)



noise_param(tau_out_a_x, oadev_a_x)
noise_param(tau_out_a_y, oadev_a_y)
noise_param(tau_out_a_z, oadev_a_z)




fig, axes = plt.subplots(2, 1, figsize=(10, 8))


axes[0].loglog(tau_out_g_x, oadev_g_x, label='Gyro X')
axes[0].loglog(tau_out_g_y, oadev_g_y, label='Gyro Y')
axes[0].loglog(tau_out_g_z, oadev_g_z, label='Gyro Z')
axes[0].set_title('Allan Variance - Gyro')
axes[0].set_xlabel('Tau (s)')
axes[0].set_ylabel('Allan Deviation')
axes[0].legend()


axes[1].loglog(tau_out_a_x, oadev_a_x, label='Accel X')
axes[1].loglog(tau_out_a_y, oadev_a_y, label='Accel Y')
axes[1].loglog(tau_out_a_z, oadev_a_z ,label='Accel Z')
axes[1].set_title('Allan Variance - Accelerometer')
axes[1].set_xlabel('Tau (s)')
axes[1].set_ylabel('Allan Deviation')
axes[1].legend()

plt.tight_layout()
plt.show()




















































