#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


def log_sim_ball_pos(msg):
    open("Workspace/logger/log_sim_ball_pos.csv", "a").write(str(msg.header.stamp) + " " + str(msg.pose.position.x) + " " + str(msg.pose.position.y) + " " + str(msg.pose.position.z) + "\n")

#MAIN
if __name__ == '__main__':
    
    rospy.init_node('sv_logger', anonymous=True)
    rospy.Subscriber("/sv_simulation/tt_ball_position", PoseStamped, log_sim_ball_pos, queue_size=1)
    
    open("Workspace/logger/log_sim_ball_pos.csv", "w").write("timestamp posX posY posZ\n")
    
    rospy.spin()
