#!/usr/bin/env/python
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32, Float64MultiArray
import message_filters

psi, psi_d, Lpwm, Rpwm = 0, 0, 0, 0


def callback(data1, data2, data3):
    global psi, psi_d, Lpwm, Rpwm
    psi = data1.data
    if(psi > 180): psi -= 360
    psi_d = data2.data
    Lpwm, Rpwm = data3.data



############################################################################
fig = plt.figure(figsize=(10,7)) 

ax = plt.subplot(221, xlim=(0, 50), ylim=(-180, 180))
ax2 = plt.subplot(222, polar = True)
ax3 = plt.subplot(212, xlim=(0, 50), ylim=(1000, 2000))
ax3.set_title("PWM Signal")

max_points = 50

line_psi, = ax.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1, c='black',ms=1 ,label="PSI")
line_psi_d, = ax.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,c='g', ms=1, label="PSI_D")

line_lpwm, = ax3.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1, c='r',ms=1, label="L_PWM")
line_rpwm, = ax3.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,c='b', ms=1, label="R_PWM")


def animate(i):
    y = psi
    old_y = line_psi.get_ydata()
    new_y = np.r_[old_y[1:], y]
    line_psi.set_ydata(new_y)

    y_2 = psi_d
    old_y_2 = line_psi_d.get_ydata()
    new_y_2 = np.r_[old_y_2[1:], y_2]
    line_psi_d.set_ydata(new_y_2)
    ax.legend(loc=3)
    return line_psi, line_psi_d

def animate2(i):
    p = psi
    p_d = psi_d

    if(p < 0): p+=360
    if(p_d<0): p_d+=360

    ax2.cla()
    ax2.set_theta_zero_location('N') 
    ax2.set_theta_direction(-1)
    ax2.axis([-math.pi, math.pi, 0, 1])
    ax2.set_thetagrids(np.arange(-180,180,45))
    ax2.set_rgrids([-1])
    
    ax2.plot([0,psi*math.pi/180],[0,1], color='gray', markersize = 1, label = "PSI")
    ax2.plot([0,psi_d*math.pi/180],[0,1], color='g', markersize = 1, label = "PSI_D")


    ax2.legend(bbox_to_anchor=(0, 0.2))
    
def animate3(i):
    y = 3000-Lpwm
    old_y = line_lpwm.get_ydata()
    new_y = np.r_[old_y[1:], y]
    line_lpwm.set_ydata(new_y)

    y_2 = Rpwm
    old_y_2 = line_rpwm.get_ydata()
    new_y_2 = np.r_[old_y_2[1:], y_2]
    line_rpwm.set_ydata(new_y_2)
    ax3.legend(loc=3)
    return line_lpwm, line_rpwm
if __name__ == '__main__':
    rospy.init_node('PWM_PLOT', anonymous=False)
    sub1 = message_filters.Subscriber("/IMUData", Float32)
    sub2 = message_filters.Subscriber("/Psi_d", Float32)
    sub3 = message_filters.Subscriber("/PWM", Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)


anim = FuncAnimation(fig, animate  ,frames=200, interval=50, blit=False)
anim2 = FuncAnimation(fig, animate2 ,frames=200, interval=50, blit=False)
anim3 = FuncAnimation(fig, animate3  ,frames=200, interval=50, blit=False)
plt.show()