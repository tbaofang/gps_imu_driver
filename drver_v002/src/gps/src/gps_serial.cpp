#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

using namespace std;

serial::Serial ser;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");

    string port;
    int baud_rate;
    if (!nh_private_.getParam("port", port))
        ROS_ERROR("could not get port!!!!!");
    if (!nh_private_.getParam("baud_rate", baud_rate))
        ROS_ERROR("could not get baud_rate!!!!!");
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("gps_serial", 1000);

    try
    {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(50);

    string str;
    while (ros::ok())
    {
        std_msgs::String result;
        string read;

        read = ser.read(ser.available());
        // cout << read.size() << endl;
        // read = "$GPFPD,0,619.600,360.000,1.675,-0.281,0.0000000,0.0000000,0.00,0.000,0.000,0.000,0.000,0,0,0C*1A$GTIMU,0,619.600,-0.2576,-0.5821,0.0620,0.0049,0.0292,0.9982,32.0*65";

        string foundFPD = "$GPFPD";
        string foundIMU = "$GTIMU";

        if (read.size() != 0)
        {
            str.append(read);
            if (str.size() > 400)
            {
                unsigned int loc1 = str.find(foundFPD, 0);
                unsigned int loc2 = str.find(foundIMU, loc1 + 1);
                unsigned int loc3 = str.find(foundFPD, loc2 + 1);
                // if (loc1 >= 399 || loc2 >= 399 || loc3 > 399)
                // {
                //     str.erase();
                //     continue;
                // }
                if(loc1 > 400){
                    ROS_INFO("read GPS fail!!!!");
                    str.erase();
                    continue;
                }
                if(loc2 > 400){
                    ROS_INFO("read IMU fail!!!!");
                    ser.write("$cmd,output,com1,gtimu,0.1*ff");
                    ROS_ERROR("write imu contrl command succeed!");
                    str.erase();
                    continue;
                }
                if(loc3 > 400){
                    str.erase();
                    continue;
                }
                string gpfpd(str, loc1, loc2 - loc1);
                string gtimu(str, loc2, loc3 - loc2);
                result.data = gpfpd + "," + gtimu;
                read_pub.publish(result);
                str.erase(0, loc3);
                // cout << "loc1 :" << loc1 << " loc2: " << loc2 <<" loc3: " << loc3 << endl;
                // cout << "[str.size:] " << str.size() << endl;
                // cout << "[gpfpd:] " << gpfpd << endl;
                // cout << "[gtimu:] " << gtimu << endl;
                // cout << "[result:] " << result.data.c_str() << endl << endl;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
