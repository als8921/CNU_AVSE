#!/usr/bin/env/python
import rospy
import message_filters
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray, Float32

lidardata = []

ax = plt.subplot(111, polar=True)


data_range = np.linspace(0,2*np.pi, 360)
plt.gcf().set_facecolor((0, 0.5, 1, 0.1))

def callback(data):
    global lidardata
    lidardata = data.data

def animate(i):
    lidarinv = [0] * 360
    ld = list(lidardata)
    for i in range(61, 300):
        ld[i] = 0
    for i in range(360):
        if(ld[i] == 0):
            lidarinv[i] = 1000


    ax.clear()
    ax.set_theta_zero_location('N') 
    ax.set_theta_direction(-1)  
    ax.set_facecolor("aliceblue")
    ax.plot(data_range, ld, 'o', color='r', markersize = 1)
    ax.fill(data_range,ld, color = (0, 1, 0.5, 0.4))
    ax.fill(data_range,lidarinv,'0.9')
    ax.plot(0,0,'s', color = "white", markersize = 1)
    ax.arrow(0, 0, 0, 15, alpha = 0.5, width = 5, edgecolor = 'blue', facecolor = 'blue', lw = 2)
    
    ax.set_thetamin(-90)
    ax.set_thetamax(90) 
    ax.set_rmax(500)
    ax.set_rmin(0)




if __name__=="__main__":
    rospy.init_node('LidarPython',anonymous = False)
    sub = message_filters.Subscriber("/LidarData", Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)


    ani = FuncAnimation(plt.gcf(), animate, interval = 50)
    plt.show()
