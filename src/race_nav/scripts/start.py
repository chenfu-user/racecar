#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import os,time

def open_terminal(commands):
    cmd_list = []
    for cmd in commands:
        cmd_list.append(""" gnome-terminal --tab -e "bash -c '%s;exec bash'" >/dev/null  2>&1 """ %cmd)

    os.system(';'.join(cmd_list))

def launch_pkg():
    '''
    启动必要的程序
    '''
    nav_cmd = [
        'roslaunch racecar_description racecar.launch', 
        'sleep 5; roslaunch race_nav race_nav.launch',
        'sleep 6; roslaunch art_racecar Run_gmapping.launch',
        #'sleep 7;cd ~/racecar_ws/src/race_nav/scripts && python key_point.py',
        'sleep 8;cd ~/racecar_ws/src/race_nav/scripts && python send_goal.py'
    ]
    open_terminal(nav_cmd)
    # open_terminal(cmd)
    #time.sleep(1)


if __name__  == '__main__':
    try:
        launch_pkg()
    except KeyboardInterrupt:
        print('操作已取消')
        os._exit()

