#!/usr/bin/env/python
import rospy
import numpy as np
import message_filters
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray

plot= np.zeros((100,100))
fig, ax = plt.subplots(figsize = (20,10))

sin_theta = np.array([math.sin(i *math.pi / 180) for i in range(360)])
cos_theta = np.array([math.cos(i *math.pi / 180) for i in range(360)])

def callback(data):
    global lidardata
    lidardata = np.array(data.data)

# plot = np.array([[0 for i in range(360)] for j in range(360)])
def animate(i):

    ax.clear()
    ax.axis([-1000,1000,0,1000])
    ax.plot(lidardata * sin_theta, lidardata * cos_theta, 'o', color='r', markersize = 10)
    ax.plot(0,0,'s', color = "blue", markersize = 15)



if __name__=="__main__":
    rospy.init_node('LidarPython',anonymous = False)
    sub = message_filters.Subscriber("/LidarData", Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)


    ani = FuncAnimation(fig, animate, interval = 50)
    plt.show()