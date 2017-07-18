#!/usr/bin/env python

import rospy, os
from gazebo_msgs.srv import DeleteModel

if __name__ == '__main__':

    rospy.init_node("table_tennis_ball_remove")
    rospy.sleep(1)

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/delete_model")
    print("delete_model service ok")

    #delete_model = rospy.ServicePoxy("gazebo/delete_model", DeleteModel)
    
    #delete_model("table_tennis_ball")
    os.system("rosservice call gazebo/delete_model \'{model_name: table_tennis_ball}\'")
    print("object deleted")
