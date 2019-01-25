#include <iostream>
#include <cstring>
#include <vector>
// #include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;

ros::Publisher fix_pub;
ros::Publisher imu_pub;
ros::Publisher twist_pub;
// ros::Publisher velocity_pub;
// ros::Publisher angular_pub;

vector<string> split(const string& str, const string& delim){
    vector<string> res;
    if("" == str) return res;
    // ROS_INFO("%s",str.c_str());
    char * strs = new char[str.length() + 1];
    strcpy(strs, str.c_str());
    char * d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char * p = strtok(strs, d);
    while(p){
        string s = p;
        res.push_back(s);
        p = strtok(NULL, d);
    }

    return res;
}




void gpsCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("receive: %s", msg->data.c_str());
    if(msg->data.c_str() == "")
    {
        cout << "subscribe is empty!" << endl;
    }

    vector<string> res = split(msg->data, ",");
    vector<string> status = split(res[15], "*");

    // for(int i = 0; i < res.size(); ++i){
    //     cout << "(" << i << " " << res[i] << ")" << "   ";
    // }
    // cout << endl << endl;
    // for(int i = 0; i < status.size(); ++i){
    //     cout << "(" << i << " " << status[i] << ")" << "   ";
    // }
    // cout << endl << endl << endl;


    // parse $GPFPD   
    if(res.size() > 15)
    {
        sensor_msgs::NavSatFix fix;
        sensor_msgs::Imu imu;
        geometry_msgs::Twist twist;

        //fix
        fix.header.stamp = ros::Time::now();
        fix.header.frame_id = "gps";
        fix.latitude = atof(res[6].c_str());
        fix.longitude = atof(res[7].c_str());
        fix.altitude = atof(res[8].c_str());

        if(status[0] == "0A")
            fix.status.status = 10;
        else if(status[0] == "0B")
            fix.status.status = 11;
        else if(status[0] == "0C")
            fix.status.status = 12;
        else
            fix.status.status = atoi(status[0].c_str());
        fix_pub.publish(fix);

        //imu
        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "imu";
        
        double roll = atof(res[5].c_str());
        double pitch = atof(res[4].c_str());
        double yaw = atof(res[3].c_str());
        //Euler conversion quaternion
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        imu.orientation = quat;
        imu.angular_velocity.x = atof(res[19].c_str());
        imu.angular_velocity.y = atof(res[20].c_str());
        imu.angular_velocity.z = atof(res[21].c_str());
        imu.linear_acceleration.x = atof(res[22].c_str());
        imu.linear_acceleration.y = atof(res[23].c_str());
        imu.linear_acceleration.z = atof(res[24].c_str());
        imu_pub.publish(imu);

        //twist
        twist.linear.x = atof(res[9].c_str());
        twist.linear.y = atof(res[10].c_str());
        twist.linear.z = atof(res[11].c_str());
        twist_pub.publish(twist);
    }




    // parse $GPRMC
    // if(res.size() > 6)
    // {
    //     sensor_msgs::NavSatFix fix;
    //     fix.header.stamp = ros::Time::now();
    //     if("N" == res[4].c_str()){
    //         fix.latitude = atof(res[3].c_str());
    //     }
    //     else{
    //         fix.latitude = -atof(res[3].c_str());
    //     }
    //     if("E" == res[6].c_str()){
    //         fix.longitude = atof(res[5].c_str());
    //     }
    //     else{
    //         fix.longitude = -atof(res[5].c_str());
    //     }
           
    //     fix_pub.publish(fix);
    //     //ROS_INFO("ok");
    // }


}

int main(int argc, char** argv){
    ros::init(argc, argv, "gps_parse");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("gps_serial", 1000, gpsCallback);

    fix_pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 1000);
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    twist_pub = nh.advertise<geometry_msgs::Twist>("twist", 1000);
    ros::spin();

}
