#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf2_ros/transform_listener.h" 
#include "tf/transform_datatypes.h"
#include <cmath>
#include <sv_effectors/HeadAngles.h>

    ros::ServiceClient moveHead_client;
    tf2_ros::TransformListener *tfListener;
    tf2_ros::Buffer tfBuffer;

    void effectorCallback(const geometry_msgs::Pose look_at_point_msg)
    {
        geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform("world", "head_kinect_rgb_optical_frame", ros::Time(0));
        geometry_msgs::Transform world_head_transform_msg = ts.transform;

        tf::Transform world_head_transform;
        tf::transformMsgToTF(world_head_transform_msg, world_head_transform);
        tf::Vector3 vect1 = world_head_transform.getOrigin();

        tf::Transform look_at_point_transform;
        tf::poseMsgToTF(look_at_point_msg, look_at_point_transform);
        tf::Vector3 vect2 = look_at_point_transform.getOrigin();
        
        tf::Transform head_object_transform = look_at_point_transform.inverse() * world_head_transform;
        tf::Vector3 vect3 = head_object_transform.getOrigin();
        
        sv_effectors::HeadAngles srv;
        srv.request.alfa = atan2(vect3.getX(), -vect3.getZ());
        srv.request.beta = -atan2(vect3.getY(), -vect3.getZ());
        
        moveHead_client.call(srv);
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_system");
    ros::NodeHandle nh("~");
    
    std::string service;
    nh.getParam("service", service);
    moveHead_client = nh.serviceClient<sv_effectors::HeadAngles>(service);

    std::string subscriber;
    nh.getParam("subscriber", subscriber);
    ros::Subscriber sub = nh.subscribe(subscriber, 1, effectorCallback);

    tfListener = new tf2_ros::TransformListener(tfBuffer);
    ros::Duration(1).sleep();

    ros::spin();
}
