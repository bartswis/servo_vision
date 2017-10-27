#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


def initialize():
    
    marker_sim_ball.header.frame_id = "/world"
    marker_sim_ball.type = marker_sim_ball.SPHERE
    marker_sim_ball.action = marker_sim_ball.ADD
    marker_sim_ball.scale.x = 0.04
    marker_sim_ball.scale.y = 0.04
    marker_sim_ball.scale.z = 0.04
    marker_sim_ball.color.r = 1.0
    marker_sim_ball.color.g = 1.0
    marker_sim_ball.color.b = 0.0
    marker_sim_ball.color.a = 1.0
    marker_sim_ball.id = 123;
    
    marker_sys_ball.header.frame_id = "/world"
    marker_sys_ball.type = marker_sys_ball.SPHERE
    marker_sys_ball.action = marker_sys_ball.ADD
    marker_sys_ball.scale.x = 0.04
    marker_sys_ball.scale.y = 0.04
    marker_sys_ball.scale.z = 0.04
    marker_sys_ball.color.r = 1.0
    marker_sys_ball.color.g = 0.0
    marker_sys_ball.color.b = 0.0
    marker_sys_ball.color.a = 1.0
    marker_sys_ball.id = 124;
    
    marker_sys_ball_fil.header.frame_id = "/world"
    marker_sys_ball_fil.type = marker_sys_ball_fil.SPHERE
    marker_sys_ball_fil.action = marker_sys_ball_fil.ADD
    marker_sys_ball_fil.scale.x = 0.04
    marker_sys_ball_fil.scale.y = 0.04
    marker_sys_ball_fil.scale.z = 0.04
    marker_sys_ball_fil.color.r = 1.0
    marker_sys_ball_fil.color.g = 0.0
    marker_sys_ball_fil.color.b = 1.0
    marker_sys_ball_fil.color.a = 1.0
    marker_sys_ball_fil.id = 125;
    

def mark_sim_ball(msg):
    
    marker_sim_ball.pose.position.x = msg.pose.position.x
    marker_sim_ball.pose.position.y = msg.pose.position.y
    marker_sim_ball.pose.position.z = msg.pose.position.z
    
    publish_mark_sim_ball.publish(marker_sim_ball)


def mark_sys_ball(msg):
    
    marker_sys_ball.pose.position.x = msg.pose.position.x
    marker_sys_ball.pose.position.y = msg.pose.position.y
    marker_sys_ball.pose.position.z = msg.pose.position.z
    publish_mark_sys_ball.publish(marker_sys_ball)
    

def mark_sys_ball_fil(msg):
    
    marker_sys_ball_fil.pose.position.x = msg.pose.position.x
    marker_sys_ball_fil.pose.position.y = msg.pose.position.y
    marker_sys_ball_fil.pose.position.z = msg.pose.position.z
    
    publish_mark_sys_ball_filtered.publish(marker_sys_ball_fil)


#MAIN
if __name__ == '__main__':

    rospy.init_node('sv_markers', anonymous=True)
    rospy.sleep(1)
    
    marker_sim_ball = Marker()
    marker_sys_ball = Marker()
    marker_sys_ball_fil = Marker()
    initialize()
    
    rospy.Subscriber("/sv_simulation/tt_ball_position", PoseStamped, mark_sim_ball, queue_size=1)
    rospy.Subscriber("/sv_control_system/control_system/ball_position", PoseStamped, mark_sys_ball, queue_size=1)
    rospy.Subscriber("/sv_control_system/control_system/ball_position_filtered", PoseStamped, mark_sys_ball_fil, queue_size=1)
    
    publish_mark_sim_ball = rospy.Publisher('marker_simulation_ball', Marker, queue_size = 1)
    publish_mark_sys_ball = rospy.Publisher('marker_system_ball', Marker, queue_size = 1)
    publish_mark_sys_ball_filtered = rospy.Publisher('marker_system_ball_filtered', Marker, queue_size = 1)
    
    rospy.spin()
