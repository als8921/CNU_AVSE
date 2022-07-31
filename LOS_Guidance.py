import rospy
import message_filters
import math
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import NavSatFix

### Parameter ###
end_range=0.00004
Delta=0.00002
##
WP_Past = []
k=0
def update(data1, data2):
    global k, WP_Past
    Waypoint=data2.data
    WP = [0] * len(Waypoint)
    for i in range(0,len(Waypoint), 2):
        WP[i], WP[i+1] = Waypoint[i+1],Waypoint[i]

    if(WP_Past != WP):
        k = 0
        print("Changed")

    WP_Past = WP

    # x=data1.longitude
    # y=data1.latitude

    x, y = data1.data[0], data1.data[1]

    if (k <= len(WP) - 4):

        pi_p = math.atan2(WP[k + 2] - WP[k], WP[k + 3] - WP[k + 1])
        y_e = (x - WP[k]) * math.cos(pi_p) - (y - WP[k + 1]) * math.sin(pi_p)
        pi_p = pi_p * 180 / math.pi
        psi_d = pi_p - math.atan(y_e / Delta) * 180 / math.pi
        if(math.pow(WP[k + 2] - x, 2) + math.pow(WP[k + 3] - y, 2) < end_range * end_range):
            k += 2

    else:
        psi_d = -10000


    if psi_d > 180:
        psi_d -= 360
    pubdata = Float32()
    pubdata.data = psi_d
    # print(psi_d)
    pub.publish(pubdata)



if __name__=="__main__":
    rospy.init_node("LOS_Guidance", anonymous=False)
    pub = rospy.Publisher("/Psi_d", Float32, queue_size=10)
    # sub1 = message_filters.Subscriber("/ublox_gps/fix", NavSatFix)
    sub1 = message_filters.Subscriber("/GPSData", Float64MultiArray)
    sub2 = message_filters.Subscriber("/Waypoint", Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 10, 0.1, allow_headerless=True)
    mf.registerCallback(update)
    rospy.spin()