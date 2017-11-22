#!/usr/bin/env python

import rospy, rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *

if __name__ == '__main__':

    rospy.init_node("table_tennis_ball_spawn")
    rospy.sleep(1)

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    print("spawn_urdf_model service ok")

    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    
    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('sv_simulation')

    with open(pack_path+"/data/objects/table_tennis_ball/model.urdf", "r") as f:
        product_xml = f.read()
        
    ball_pose = Pose()
    ball_pose.position.x = 0.75
    ball_pose.position.y = 0.0
    ball_pose.position.z = 1.5
    ball_pose.orientation.x = 0
    ball_pose.orientation.y = 0
    ball_pose.orientation.z = 0
    ball_pose.orientation.w = 1

    item_pose = Pose(Point(x=1, y=0, z=1.5), Quaternion(0,0,0,1))
    spawn_model("table_tennis_ball", product_xml, "", ball_pose, "world")
    print("object spawned")
