#!/usr/bin/env python
# coding=utf-8
import math
import actionlib
import dynamic_reconfigure.client as dyclient
import rosparam
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

key_points = [
              [4.40,0,1.0,0.77,1.557],
              [8.59,0,0,1.045,1.532],
             # [19.2,1.65,0,1.025,1.54],
              [8.62,1.84,0,0.76,1.55],
              [4.46,1.835,0,1.1,1.55],
              [-1.2,1.9,0,0.0,0.3]
             ] #每个列表第一个数值为x坐标，第二个：y，第四个：更新后的x方向速度,第五个为角加速度

PASS_THRES_RADIUS = 0.50

def is_passed(now_pos, next_waypoint):
    dis_to_next_point = math.sqrt(
        (now_pos[0]-next_waypoint[0])**2+(now_pos[1]-next_waypoint[1])**2)
    # print('--dis:%.2f %.2f'%(dis_to_next_point[0],dis_to_next_point[1]))
    if dis_to_next_point <= PASS_THRES_RADIUS:
        return True
    else:
        return False

class Update_cfg():

    def __init__(self):
        self.x = 0
        self.y = 0
        self.odom_sub = 0
        self.reconfig_client = dyclient.Client('/move_base/TebLocalPlannerROS')

    def get_odom_data(self, odom_data):
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y


if __name__ == '__main__':
    rospy.init_node('keypoints')

    rospy.wait_for_service("/move_base/TebLocalPlannerROS/set_parameters")
    update_cfg = Update_cfg()
    
    rospy.Subscriber("/odom", Odometry, update_cfg.get_odom_data)

    params = {'max_vel_x': 1.2, 'max_vel_theta': 7.0,'acc_lim_theta':3.0}
    config = update_cfg.reconfig_client.update_configuration(params)
    print('begin!!!')

    try:
        for i in range(len(key_points)):
            while not is_passed((update_cfg.x, update_cfg.y), (key_points[i][0], key_points[i][1])):
                pass
            r = rospy.Rate(0.1)
            # 经过关键点后更新速度限制
            params = {'max_vel_x': key_points[i][3],'acc_lim_theta':key_points[i][4]}
            config = update_cfg.reconfig_client.update_configuration(params)
            print("--passed point [%d]" % (i+1))
        print('--结束')

    except KeyboardInterrupt:
        print('操作已取消')
        exit(0)

