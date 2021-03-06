#!/usr/bin/env python
import rospy
import message_filters
import time
from std_msgs.msg import Float64MultiArray, Float32

psi_error_past = 0

timepast = time.time()

P_gain = 2
D_gain = 0.5

tau_N = 0
tau_X = 350

minThrust = 1050
maxThrust = 1950

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

    tempThrust = tau_X + abs(tau_N * 0.5) + 1500
    if(tempThrust > maxThrust):
        if(tau_N > 0):
            Lpwm, Rpwm = maxThrust, maxThrust - tau_N
        if(tau_N < 0):
            Lpwm, Rpwm = maxThrust + tau_N, maxThrust
    else:
        Rpwm = tau_X - tau_N / 2 + 1500
        Lpwm = tau_X + tau_N / 2 + 1500

    if(isEnd):
        Lpwm, Rpwm = 1500, 1500

    if(Rpwm > maxThrust): Rpwm = maxThrust
    elif(Rpwm < minThrust) : Rpwm = minThrust

    if(Lpwm > maxThrust): Lpwm = maxThrust
    elif(Lpwm < minThrust) : Lpwm = minThrust    

    Lpwm = 3000 - Lpwm
    PWM_data = Float64MultiArray()
    PWM_data.data = [Lpwm, Rpwm]
    print("PSI : ", data1.data," PSI_D : ",data2.data, " Lpwm : ", Lpwm, " Rpwm : ", Rpwm)
    pub.publish(PWM_data)
    timepast = time.time()
    


if __name__ == '__main__':
    rospy.init_node('PWM_Control', anonymous=False)
    pub = rospy.Publisher('PWM', Float64MultiArray, queue_size=10)
    sub1 = message_filters.Subscriber("/IMUData", Float32)
    sub2 = message_filters.Subscriber("/Psi_d", Float32)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)
    rospy.spin()