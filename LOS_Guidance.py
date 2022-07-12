import rospy
import message_filters
import math
from std_msgs.msg import Float32, Float64MultiArray

### Parameter ###
end_range=50
Delta=50
##

k=0
def update(data1, data2):
    global k
    WP=data2.data
    x=data1.data[0]
    y=data1.data[1]

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
    pub.publish(pubdata)



if __name__=="__main__":
    rospy.init_node("LOS_Guidance")

    pub = rospy.Publisher("/Psi_d", Float32, queue_size=10)
    sub1 = message_filters.Subscriber("/GPSData", Float64MultiArray)
    sub2 = message_filters.Subscriber("/Waypoint", Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 10, 0.1, allow_headerless=True)
    mf.registerCallback(update)
    rospy.spin()