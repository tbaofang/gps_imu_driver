//
// Created by tbf on 18-11-21.
//
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

using namespace std;

ros::Publisher pose_pub;



void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    assert(imu);

    geometry_msgs::Pose pose;
    pose.orientation = imu->orientation;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu->orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    cout << "roll=" << roll << " " << "pitch=" << pitch << " " << "yaw=" << yaw << endl;

    pose_pub.publish(pose);




//    geometry_msgs::PoseStamped pose_stamped;
//    nav_msgs::Path path;
//    path.header.frame_id = "/imu";
//    path.header.stamp = imu->header.stamp;
//
//    tf::Quaternion quat;
//    tf::quaternionMsgToTF(imu->orientation, quat);
//    double roll, pitch, yaw;
//    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//    cout << "roll=" << roll << " " << "pitch=" << pitch << " " << "yaw=" << yaw << endl;
//
//
//    pose_stamped.pose.orientation = imu->orientation;
//
//    path.poses.push_back(pose_stamped);
//    path_pub.publish(path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_show_path");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("imu", 100, imuCallback);

    pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 100);
    ros::spin();
}
