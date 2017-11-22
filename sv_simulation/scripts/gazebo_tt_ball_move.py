#!/usr/bin/env python

import rospy, math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, PoseStamped

def move(time_, mvX, mvY, mvZ):
    
    ball_pose.position.x += mvX
    ball_pose.position.y += mvY
    ball_pose.position.z += mvZ
     
    state = ModelState()
    state.model_name = "table_tennis_ball"
    state.pose = ball_pose
    
    out_msg = PoseStamped()
    out_msg.header.frame_id = "/tt_ball"
    out_msg.header.stamp = rospy.Time.now()
    out_msg.pose = ball_pose
    ret = set_model_state(state)
    pub.publish(out_msg)
       
    print ret.status_message
    rospy.sleep(time_)

if __name__ == '__main__':

    rospy.init_node("table_tennis_ball_object")
    rospy.sleep(1)

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/set_model_state")
    print("set_model_state service ok")

    set_model_state = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
    
    pub = rospy.Publisher('sv_simulation/tt_ball_position', PoseStamped, queue_size=1)

    global ball_pose
    ball_pose = Pose()
    ball_pose.position.x = 0.75
    ball_pose.position.y = 0.0
    ball_pose.position.z = 1.5
    ball_pose.orientation.x = 0
    ball_pose.orientation.y = 0
    ball_pose.orientation.z = 0
    ball_pose.orientation.w = 1
    
    sp = 300
    for i in range(0,2*sp,1):
        move(0.01, 0.0, math.sin(i*math.pi/sp)/sp, math.cos(i*math.pi/sp)/sp)
    for i in range(2*sp,0,-1):
        move(0.01, 0.0, math.sin(i*math.pi/sp)/sp, math.cos(-i*math.pi/sp)/sp)
    '''
    '''
    sp = 200
    for i in range(0,2*sp,1):
        move(0.01, -math.sin(i*math.pi/sp)/sp/2, math.cos(i*math.pi/sp)/sp/2, 0.0)
    '''
    '''
    for i in range(0,2*sp,1):
        move(0.01, -math.sin(i*math.pi/sp)/sp/2, 0.0, math.cos(i*math.pi/sp)/sp/2)
