#! /usr/bin/env python2
# -*- coding: utf-8 -*-

from ros_module import ROSNavNode
import rospy
import os
import time

def send():
    node = ROSNavNode()                                              # 启动节点
    start = node.curr_time.secs
    n_start = node.curr_time.nsecs
    print('开始发送目标')
    node.send_goal()


    end = node.curr_time.secs
    n_end = node.curr_time.nsecs
    cost = end - start

    if n_end>n_start:
        n_cost = n_end - n_start
    else:
        n_cost = n_end + 1000000000 - n_start

    n_cost = float(n_cost) / 1000000000.0



if __name__  == '__main__':
    try:
        send()
    except KeyboardInterrupt:
        print('\node')
        print('操作已取消')
        os._exit()
