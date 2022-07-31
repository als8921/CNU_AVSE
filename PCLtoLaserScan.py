import rospy
import ros_numpy
import numpy as np

from sensor_msgs.msg import PointCloud2, LaserScan

ls = LaserScan()
ls.header.frame_id = "rslidar"
ls.angle_increment = np.pi / 180.0
ls.angle_min = 0
ls.angle_max = 359 * np.pi / 180.0
ls.range_min = 0
ls.range_max = 20

def callback(data):
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    theta = []
    print(len(xyz_array))
    for x, y, z in xyz_array:
        theta.append(np.round(np.arctan2(y, x)*57.295791))

    xyz_array = np.square(xyz_array)
    r = np.sqrt(xyz_array.sum(axis=1))
    theta = np.array(theta)

    lidardata = np.zeros(360)

    for i in range(len(theta)):
        if(lidardata[int(theta[i])]==0 or lidardata[int(theta[i])] > r[i]):
            lidardata[int(theta[i])] = r[i]

    ls.ranges = lidardata
    pub.publish(ls)


rospy.init_node("PCL")
pub = rospy.Publisher("/LidarLaserScan",LaserScan, queue_size=10)
sub = rospy.Subscriber("/voxel_grid/output", PointCloud2, callback)
rospy.spin()
