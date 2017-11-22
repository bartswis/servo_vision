#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "tf2_ros/transform_listener.h" 
#include "tf/transform_datatypes.h"
#include <cmath>
#include <queue>
#include <vector>
#include <ex_kalman_sv.h>

    ros::Publisher pub_position;
    ros::Publisher pub_position_filtered;
    ros::Publisher pub_position_prediction;
    ros::Publisher pub_look_at;
    
    tf2_ros::TransformListener *tfListener;
    tf2_ros::Buffer tfBuffer;

    void controlCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_object_pose_msg,
                         boost::shared_ptr<ExtendKalmanFilter> filter_x_ptr,
                         boost::shared_ptr<ExtendKalmanFilter> filter_y_ptr,
                         boost::shared_ptr<ExtendKalmanFilter> filter_z_ptr)
    {
        geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform("world", "head_kinect_rgb_optical_frame", ros::Time(0));
        geometry_msgs::Transform world_camera_transform_msg = ts.transform;
        
        //ROS_ERROR("%f %f %f", world_camera_transform_msg.translation.x, world_camera_transform_msg.translation.y, world_camera_transform_msg.translation.z);
        //ROS_ERROR("%f %f %f %f", world_camera_transform_msg.rotation.x, world_camera_transform_msg.rotation.y, world_camera_transform_msg.rotation.z, world_camera_transform_msg.rotation.w);

        tf::Transform world_camera_transform;
        tf::transformMsgToTF(world_camera_transform_msg, world_camera_transform);

        tf::Transform camera_object_transform;
        tf::poseMsgToTF(camera_object_pose_msg->pose, camera_object_transform);

        tf::Transform world_object_transform = world_camera_transform * camera_object_transform;
        
        geometry_msgs::PoseStamped world_object_pose;
        tf::poseTFToMsg(world_object_transform, world_object_pose.pose); 
        world_object_pose.header.stamp = ros::Time::now();
        pub_position.publish(world_object_pose);
        
        geometry_msgs::PoseStamped world_object_pose_filtered;
        world_object_pose_filtered.pose = world_object_pose.pose;
        world_object_pose_filtered.pose.position.x = filter_x_ptr->compute(world_object_pose.pose.position.x);
        world_object_pose_filtered.pose.position.y = filter_y_ptr->compute(world_object_pose.pose.position.y);
        world_object_pose_filtered.pose.position.z = filter_z_ptr->compute(world_object_pose.pose.position.z);
        world_object_pose_filtered.header.stamp = ros::Time::now();
        pub_position_filtered.publish(world_object_pose_filtered);
        
        geometry_msgs::PoseArray world_object_pose_prediction;
        filter_x_ptr->reset();
        filter_y_ptr->reset();
        filter_z_ptr->reset();
        for (int i=0; i<50; i++)
        {
            geometry_msgs::Pose pose_tmp;
            pose_tmp = world_object_pose_filtered.pose;
            pose_tmp.position.x = filter_x_ptr->predict();
            pose_tmp.position.y = filter_y_ptr->predict();
            pose_tmp.position.z = filter_z_ptr->predict();
            world_object_pose_prediction.poses.push_back(pose_tmp);
            
        }
        world_object_pose_prediction.header.stamp = ros::Time::now();
        pub_position_prediction.publish(world_object_pose_prediction);
        
        geometry_msgs::Pose look_at_point;
        look_at_point = world_object_pose_filtered.pose;
        pub_look_at.publish(look_at_point);
        
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_system");
    ros::NodeHandle nh("~");

    std::string publisher_pos;
    nh.getParam("publisher_pos", publisher_pos);
    pub_position = nh.advertise<geometry_msgs::PoseStamped>(publisher_pos, 1);
    pub_position_filtered = nh.advertise<geometry_msgs::PoseStamped>(publisher_pos+"_filtered", 1);
    pub_position_prediction = nh.advertise<geometry_msgs::PoseArray>(publisher_pos+"_prediction", 1);
    
    std::string publisher_cmd;
    nh.getParam("publisher_cmd", publisher_cmd);
    pub_look_at = nh.advertise<geometry_msgs::Pose>(publisher_cmd, 1);
    
    boost::shared_ptr<ExtendKalmanFilter> filter_x_ptr (new ExtendKalmanFilter());
    boost::shared_ptr<ExtendKalmanFilter> filter_y_ptr (new ExtendKalmanFilter());
    boost::shared_ptr<ExtendKalmanFilter> filter_z_ptr (new ExtendKalmanFilter());

    std::string subscriber;
    nh.getParam("subscriber", subscriber);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(subscriber, 1, boost::bind(controlCallback, _1, filter_x_ptr, filter_y_ptr, filter_z_ptr));

    tfListener = new tf2_ros::TransformListener(tfBuffer);
    ros::Duration(1).sleep();

    ros::spin();
}
