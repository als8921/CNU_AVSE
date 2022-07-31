#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Float32
from math import pi


def getYaw(q):
    quaternion = (q.x, q.y, q.z, q.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = -1.0 * euler[2] * 180.0/pi
    return yaw

def callback(msg):
    data = Float32()
    data.data = getYaw(msg.quaternion)
    print("Psi : ",data.data)
    pub.publish(data)


if __name__ == '__main__':
    rospy.init_node('IMU_talker', anonymous=False)
    pub = rospy.Publisher('IMUData', Float32, queue_size=100)
    rospy.Subscriber('/filter/quaternion', QuaternionStamped, callback)