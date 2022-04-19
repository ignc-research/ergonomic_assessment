#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

## helper functions
# def inv(T):
#     """invert 4x4 homogenous transform matrix"""
#     # Craig2005 - Introduction to Robotics p.36
#     T_new = np.zeros((4,4))
#     T_new[3,3] = 1.0
#     T_new[0:3, 0:3] = np.transpose(T[0:3, 0:3])
#     T_new[0:3,3] = -np.matmul(np.transpose(T[0:3, 0:3]), T[0:3,3])
#     return T_new

def norm(q):
    """return quaternion norm"""
    return np.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)

def tq_to_T(t,q):
    """convert translation t and quaternion q to 4x4 homogenous Matrix"""
    assert(norm(q) <= 1.001 and norm(q)>= 0.999)  # check for unit quaternion
    T = np.zeros((4,4))
    T[0,3] = t.x
    T[1,3] = t.y
    T[2,3] = t.z
    T[3,3] = 1.0

    # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    T[0,0] = 1 - 2*q.y*q.y - 2*q.z*q.z
    T[0,1] = 2*q.x*q.y-2*q.z*q.w
    T[0,2] = 2*q.x*q.z + 2*q.y*q.w
    T[1,0] = 2*q.x*q.y + 2*q.z*q.w
    T[1,1] = 1-2*q.x*q.x - 2*q.z*q.z
    T[1,2] = 2*q.y*q.z - 2*q.x*q.w
    T[2,0] = 2*q.x*q.z - 2*q.y*q.w
    T[2,1] = 2*q.y*q.z + 2*q.x*q.w
    T[2,2] = 1-2*q.x*q.x - 2*q.y*q.y

    return T

## IMU calibration for Microsoft Azure Kinect sensor
## requires /tf_static and /imu topics
class ImuCalibration():
    def __init__(self):
        self.x_acc = []
        self.y_acc = []
        self.z_acc = []
        self.imu_to_depth = np.zeros((4,4))
        rospy.Subscriber("/kinect2/imu", Imu, self.callbackImu)
        rospy.Subscriber("/tf_static", TFMessage, self.callbackStatic)

    def callbackImu(self, data):
        """get x,y,z linear acceleration"""
        self.x_acc.append(data.linear_acceleration.x)
        self.y_acc.append(data.linear_acceleration.y)
        self.z_acc.append(data.linear_acceleration.z)

    def callbackStatic(self, data):
        """get static transform between IMU and Depth sensor from /tf_static"""
        for d in data.transforms:
            if(d.header.frame_id == "kinect2_depth_camera_link" and d.child_frame_id == "kinect2_imu_link"):
                T = tq_to_T(d.transform.translation, d.transform.rotation)
                self.imu_to_depth = T
    
    def getVerticalVector(self):
        """compute vertical vector from IMU data and static TF Transform"""
        if not ((self.imu_to_depth == np.zeros((4,4))).all() or len(self.z_acc) < 1000):
            x_acc = np.mean(self.x_acc)
            y_acc = np.mean(self.y_acc)
            z_acc = np.mean(self.z_acc)
            acc_vec = np.array((x_acc, y_acc, z_acc, 1.0)) # (4x1) vector
            vert_vecc = np.matmul(self.imu_to_depth, acc_vec)  # (4x1) vector with linear_acceleration in depth_frame
            vert_vecc = vert_vecc[0:3]
            rospy.loginfo("Vertical Vector from IMU in {kinect2_depth_camera_link}: "+str(vert_vecc))
            rospy.loginfo("Vector magnitude: " + str(np.sqrt(x_acc**2 + y_acc**2 + z_acc**2)))  # should be around 9.81
            np.savetxt("vertical_vector.txt", vert_vecc)
            return True
        else:
            return False

def calib_start():
    rospy.init_node('ImuCalibrationNode', anonymous=True)
    calib = ImuCalibration()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            result = calib.getVerticalVector()
            if(result):
                rospy.signal_shutdown("done")
            rate.sleep()
        except KeyboardInterrupt:
            print("Shutting down")

if __name__ == '__main__':
    calib_start()