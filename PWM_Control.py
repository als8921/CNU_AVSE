#!/usr/bin/env python
import rospy
import message_filters
import time
from std_msgs.msg import Float64MultiArray, Float32

pub = rospy.Publisher('PWM', Float64MultiArray, queue_size=10)
psi_error_past = 0

timepast = time.time()

P_gain = 5
D_gain = 3

tau_N = 0
tau_X = 300
isEnd = False
def callback(data1, data2):
    global psi_error_past
    global timepast, tau_X, isEnd
    dt = time.time() - timepast
    psi = data1.data
    if(psi > 180): 
        psi-=360
    psi_d = data2.data

    if(psi_d==-10000):
        isEnd = True

    psi_error = psi_d - psi
    if(psi_error > 180): psi_error -= 360
    elif(psi_error < -180): psi_error += 360

    psi_error_dot = (psi_error-psi_error_past)/dt
    psi_error_past = psi_error

    tau_N = P_gain * psi_error + D_gain * psi_error_dot

    # if(tau_N > 2 * tau_X): tauN = 2 * tau_X
    # if(tau_N < -2 * tau_X): tau_N = -2 * tau_X

    Rpwm = tau_X - tau_N / 2 + 1500
    Lpwm = tau_X + tau_N / 2 + 1500


    if(isEnd):
        Lpwm, Rpwm = 1500, 1500

    if(Rpwm > 1950): Rpwm = 1950
    elif(Rpwm < 1050) : Rpwm = 1050

    if(Lpwm > 1950): Lpwm = 1950
    elif(Lpwm < 1050) : Lpwm = 1050    
    


    Lpwm = 3000 - Lpwm
    PWM_data = Float64MultiArray()
    PWM_data.data = [Lpwm, Rpwm]
    print(data1.data, data2.data, Lpwm, Rpwm)
    pub.publish(PWM_data)
    timepast = time.time()
    


if __name__ == '__main__':
    rospy.init_node('PPPWM', anonymous=False)
    sub1 = message_filters.Subscriber("/IMUData", Float32)
    sub2 = message_filters.Subscriber("/Psi_d", Float32)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)

    rospy.spin()

