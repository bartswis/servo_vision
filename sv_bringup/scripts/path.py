#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def initialize():

    path_sim_ball.header.frame_id = "/world"
    path_sys_ball.header.frame_id = "/world"
    path_sys_ball_fil.header.frame_id = "/world"
    path_sim_ball.poses


def pather_sim_ball(msg):

    if (len(path_sim_ball.poses) > 1000):
        path_sim_ball.poses.pop(0)
        
    path_sim_ball.poses.append(msg)
    publish_path_sim_ball.publish(path_sim_ball)


def pather_sys_ball(msg):

    if (len(path_sys_ball.poses) > 1000):
        path_sys_ball.poses.pop(0)
        
    path_sys_ball.poses.append(msg)
    publish_path_sys_ball.publish(path_sys_ball)


def pather_sys_ball_fil(msg):

    if (len(path_sys_ball_fil.poses) > 1000):
        path_sys_ball_fil.poses.pop(0)
        
    path_sys_ball_fil.poses.append(msg)
    publish_path_sys_ball_filtered.publish(path_sys_ball_fil)


#MAIN
if __name__ == '__main__':

    rospy.init_node('sv_path', anonymous=True)
    rospy.sleep(1)
    
    path_sim_ball = Path()
    path_sys_ball = Path()
    path_sys_ball_fil = Path()
    initialize()
    
    rospy.Subscriber("/sv_simulation/tt_ball_position", PoseStamped, pather_sim_ball, queue_size=1)
    rospy.Subscriber("/sv_control_system/control_system/ball_position", PoseStamped, pather_sys_ball, queue_size=1)
    rospy.Subscriber("/sv_control_system/control_system/ball_position_filtered", PoseStamped, pather_sys_ball_fil, queue_size=1)
    
    publish_path_sim_ball = rospy.Publisher('path_simulation_ball', Path, queue_size = 1)
    publish_path_sys_ball = rospy.Publisher('path_system_ball', Path, queue_size = 1)
    publish_path_sys_ball_filtered = rospy.Publisher('path_system_ball_filtered', Path, queue_size = 1)
    
    rospy.spin()
