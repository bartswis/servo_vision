#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h" 
#include "tf/transform_datatypes.h"
#include <cmath>
#include <queue>
#include <vector>
#include <sv_msg/TimeSeq.h>

    ros::Publisher pub_marker;
    ros::Publisher pub_position;
    ros::Publisher pub_delay;
    std::string marker_frame;
    tf2_ros::TransformListener *tfListener;
    tf2_ros::Buffer tfBuffer;

    void controlCallback(const geometry_msgs::PoseStamped camera_object_pose_msg)
    {
        geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform("world", "p_c_optical_frame", ros::Time(0));
        geometry_msgs::Transform world_camera_transform_msg = ts.transform;

        tf::Transform world_camera_transform;
        tf::transformMsgToTF(world_camera_transform_msg, world_camera_transform);

        tf::Transform camera_object_transform;
        tf::poseMsgToTF(camera_object_pose_msg.pose, camera_object_transform);

        tf::Transform world_object_transform = world_camera_transform * camera_object_transform;
        geometry_msgs::PoseStamped world_object_pose;
        tf::poseTFToMsg(world_object_transform, world_object_pose.pose);

        world_object_pose.header.frame_id = marker_frame;
        world_object_pose.header.stamp = ros::Time::now();
        pub_position.publish(world_object_pose);
        sv_msg::TimeSeq time_seq;
        time_seq.stamp = world_object_pose.header.stamp;
        time_seq.own_seq = camera_object_pose_msg.header.seq;
        pub_delay.publish(time_seq);

        uint32_t shape = visualization_msgs::Marker::SPHERE;
        visualization_msgs::Marker marker;
        marker.header.frame_id = marker_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 123;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = world_object_pose.pose;
        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        pub_marker.publish(marker);
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_system");
    ros::NodeHandle nh("~");

        nh.getParam("marker_frame", marker_frame);
        std::string visualization_marker;
        nh.getParam("visualization_marker", visualization_marker);
        pub_marker = nh.advertise<visualization_msgs::Marker>(visualization_marker, 1);

        std::string publisher;
        nh.getParam("publisher", publisher);
        pub_position = nh.advertise<geometry_msgs::PoseStamped>(publisher, 1);

        std::string delay;
        nh.getParam("delay", delay);
        pub_delay = nh.advertise<sv_msg::TimeSeq>(delay, 1);

        std::string subscriber;
        nh.getParam("subscriber", subscriber);
        ros::Subscriber sub = nh.subscribe(subscriber, 1, controlCallback);

        tfListener = new tf2_ros::TransformListener(tfBuffer);

    ros::spin();
}
