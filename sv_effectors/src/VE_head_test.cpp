#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "tf2_ros/transform_listener.h" 
#include "tf/transform_datatypes.h"
#include <cmath>
#include <queue>
#include <vector>
#include <sv_effectors/HeadAngles.h>

    ros::ServiceClient moveHead_client;
    tf2_ros::TransformListener *tfListener;
    tf2_ros::Buffer tfBuffer;

    void effectorCallback(const geometry_msgs::Point look_at_point_msg)
    {
        geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform("world", "head_kinect_rgb_optical_frame", ros::Time(0));
        geometry_msgs::Transform world_head_transform_msg = ts.transform;

        tf::Transform world_head_transform;
        tf::transformMsgToTF(world_head_transform_msg, world_head_transform);

        //tf::Transform look_at_point_transform;
        //tf::pointMsgToTF(look_at_point_msg, look_at_point_transform);

        //tf::Transform world_object_transform = world_camera_transform * camera_object_transform;
        
        /*
        geometry_msgs::PoseStamped world_object_pose;
        tf::poseTFToMsg(world_object_transform, world_object_pose.pose);                    
        world_object_pose.header.frame_id = marker_frame;
        world_object_pose.header.stamp = ros::Time::now();
        pub_position.publish(world_object_pose);
        */
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_system");
    ros::NodeHandle nh("~");
    
    moveHead_client = nh.serviceClient<sv_effectors::HeadAngles>("moveHead");

    std::string subscriber;
    nh.getParam("subscriber", subscriber);
    ros::Subscriber sub = nh.subscribe(subscriber, 1, effectorCallback);

    tfListener = new tf2_ros::TransformListener(tfBuffer);

    ros::spin();
}
