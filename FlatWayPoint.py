import math
import numpy as np
import matplotlib.pyplot as plt
import rospy
import message_filters
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32, Float64MultiArray

##### Setting #####
MapInfo = np.zeros([4,2])
MapInfo[0] = [36.3687312, 127.3452688]
MapInfo[1] = [36.368870, 127.345281]
MapInfo[2] = [36.3688497, 127.3455597]
MapInfo[3] = [36.368698, 127.345534]

StartGpsCoor = MapInfo[0]
EndGpsCoor = MapInfo[2]

##########

def LLh2Flat(start_p, present_p):
    start_p = [math.radians(start_p[0]), math.radians(start_p[1])]
    present_p = [math.radians(present_p[0]), math.radians(present_p[1])]
    re = 6378137
    rp = 6356752.314245
    e = 0.0818
    RN = re/math.sqrt(1-e*e*math.sin(start_p[0])*math.sin(start_p[0]))
    RM = RN * (1-e*e) / math.sqrt(1-e*e *
    math.sin(start_p[0])*math.sin(start_p[0]))
    Nned = (present_p[0] - start_p[0])/math.atan2(1, RM)
    Ened = (present_p[1] - start_p[1])/math.atan2(1, RN*math.cos(start_p[0]))
    return [Ened, Nned]

def update(data3):
    WayPoint = data3.data
    FlatWayPoint = []
    for i in range(0, len(WayPoint), 2):
        FlatWayPoint.append(LLh2Flat(StartGpsCoor, [WayPoint[i], WayPoint[i + 1]]))

    msg = Float64MultiArray()
    msg.data = np.reshape(FlatWayPoint, -1)
    pub.publish(msg)
    print(msg)
    

if __name__=="__main__":
    rospy.init_node("MAPUI")
    pub = rospy.Publisher("/FlatWayPoint", Float64MultiArray, queue_size=10)
    sub=message_filters.Subscriber('Waypoint',Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub],10, 0.1, allow_headerless=True)
    mf.registerCallback(update)
    rospy.spin()