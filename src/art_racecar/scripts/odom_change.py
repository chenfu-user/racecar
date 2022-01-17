#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class OdometryNode:
    
    def __init__(self):
        # init internals
        self.odom = Odometry()
        self.flag = 0
        self.imu_data0 = Imu()
        self.imu_data = Imu()
        self.yaw = 0

        # Set subscribers
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

        rospy.Subscriber('/ekf/odom', Odometry, self.sub_odom_update)
        rospy.Subscriber("/imu_data", Imu, self.update_posture)

    def sub_odom_update(self,msg):
        self.odom = msg
        l = quaternion_from_euler(0,0,self.yaw)
        self.odom.pose.pose.orientation.x = l[0]
        self.odom.pose.pose.orientation.y = l[1]
        self.odom.pose.pose.orientation.z = l[2]
        self.odom.pose.pose.orientation.w = l[3] 

    def update_posture(self,data):
        if self.flag == 0:
            self.imu_data0 = data
            self.flag = 1

        self.imu_data = data
        x1 = self.imu_data.orientation.x
        y1 = self.imu_data.orientation.y
        z1 = self.imu_data.orientation.z
        w1 = self.imu_data.orientation.w

        x2 = self.imu_data0.orientation.x
        y2 = self.imu_data0.orientation.y
        z2 = self.imu_data0.orientation.z
        w2 = self.imu_data0.orientation.w

        _, _, yaw1 = euler_from_quaternion([x1, y1, z1, w1])
        _, _, yaw2 = euler_from_quaternion([x2, y2, z2, w2])

        self.yaw = yaw1 - yaw2

        if self.yaw > 3.141592654:
            self.yaw = -3.141592654*2 + self.yaw
        elif self.yaw < -3.141592654:
            self.yaw = 3.141592654*2 -self.yaw

if __name__ == '__main__':
    rospy.init_node("odom_change")
    node = OdometryNode()
    while not rospy.is_shutdown():
        node.pub_odom.publish(node.odom)
        rospy.sleep(0.02)
    rospy.spin()
