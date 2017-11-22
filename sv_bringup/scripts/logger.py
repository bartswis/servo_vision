#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


def logger_sim_ball(msg):
    open("Workspace/logger/log_sim_ball.csv", "a").write(str(msg.header.stamp) + " " + str(msg.pose.position.x) + " " + str(msg.pose.position.y) + " " + str(msg.pose.position.z) + "\n")
    
def logger_sys_ball(msg):
    open("Workspace/logger/log_sys_ball.csv", "a").write(str(msg.header.stamp) + " " + str(msg.pose.position.x) + " " + str(msg.pose.position.y) + " " + str(msg.pose.position.z) + "\n")
    
def logger_sys_ball_fil(msg):
    open("Workspace/logger/log_sys_ball_fil.csv", "a").write(str(msg.header.stamp) + " " + str(msg.pose.position.x) + " " + str(msg.pose.position.y) + " " + str(msg.pose.position.z) + "\n")
    
def logger_sys_ball_pred(msg):
    open("Workspace/logger/log_sys_ball_pred.csv", "a").write(str(msg.header.stamp) + " " + str(msg.pose.position.x) + " " + str(msg.pose.position.y) + " " + str(msg.pose.position.z) + "\n")


#MAIN
if __name__ == '__main__':
    
    rospy.init_node('sv_logger', anonymous=True)
    rospy.sleep(1)
    
    rospy.Subscriber("/sv_simulation/tt_ball_position", PoseStamped, logger_sim_ball, queue_size=1)
    rospy.Subscriber("/sv_control_system/control_system/ball_position", PoseStamped, logger_sys_ball, queue_size=1)
    rospy.Subscriber("/sv_control_system/control_system/ball_position_filtered", PoseStamped, logger_sys_ball_fil, queue_size=1)
    #rospy.Subscriber("/sv_control_system/control_system/ball_position_prediction", PoseArray, logger_sys_ball_pred, queue_size=1)
    
    open("Workspace/logger/log_sim_ball_pos.csv", "w").write("timestamp posX posY posZ\n")
    open("Workspace/logger/log_sys_ball_pos.csv", "w").write("timestamp posX posY posZ\n")
    open("Workspace/logger/log_sys_ball_pos_fil.csv", "w").write("timestamp posX posY posZ\n")
    #open("Workspace/logger/log_sys_ball_pred.csv", "w").write("timestamp posX posY posZ\n")
    
    rospy.spin()
