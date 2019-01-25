#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/asio.hpp>

#include <ros/ros.h>
// #include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace boost::asio; 


unsigned char buf[4000];
size_t read_len;
vector<unsigned char> data;
vector<unsigned char> one_frame_data;
unsigned int imu_mode = 1; //0 means output raw data, 1 means processed data

ros::Publisher imu_pub;


void printE(unsigned char elem)
{
	printf("%x ", elem);
}

//Convert four bytes to float
float byte2Float(vector<unsigned char>::const_iterator pos1, vector<unsigned char>::const_iterator pos2)
{
	unsigned char byte[4];
	int i = 0;
	// for_each(pos1, pos2, printE); cout << endl;	
	while(pos1 != pos2)
	{
		byte[i++] = *pos1;
		++pos1;
	}
	for(i = 0; i < sizeof(byte)/sizeof(unsigned char); ++i)
		printf("%x ", byte[i]);
	cout << *(float*)byte << endl;
	return *(float*)byte;
	
}


void dataParseRaw(vector<unsigned char>& data)
{
	sensor_msgs::Imu imu;

	cout << endl << "one_frame_data size: " <<  data.size() << endl;
	for_each(data.begin(), data.end(), printE); cout << endl;

	vector<unsigned char>::const_iterator begin = data.begin();

	imu.header.stamp = ros::Time::now();
	imu.header.frame_id = "imu";

	imu.linear_acceleration.x = -byte2Float(begin+4, begin+8);
	imu.linear_acceleration.y = -byte2Float(begin+8, begin+12);
	imu.linear_acceleration.z = -byte2Float(begin+12, begin+16);
	imu.angular_velocity.x = -byte2Float(begin+16, begin+20);
	imu.angular_velocity.y = -byte2Float(begin+20, begin+24);
	imu.angular_velocity.z = -byte2Float(begin+24, begin+28);

	float compass_x = byte2Float(begin+28, begin+32);
	float compass_y = byte2Float(begin+32, begin+36);
	float compass_z = byte2Float(begin+36, begin+40);
//	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(compass_x, compass_y, compass_z);
//    imu.orientation = quat;

	imu_pub.publish(imu);
}

void dataParse(vector<unsigned char>& data)
{
	sensor_msgs::Imu imu;

	cout << endl << "one_frame_data size: " <<  data.size() << endl;
	for_each(data.begin(), data.end(), printE); cout << endl;

	vector<unsigned char>::const_iterator begin = data.begin();

	imu.header.stamp = ros::Time::now();
	imu.header.frame_id = "imu";

	float roll = byte2Float(begin+4, begin+8);
	float pitch = byte2Float(begin+8, begin+12);
	float yaw = byte2Float(begin+12, begin+16);
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
	imu.orientation = quat;

//	imu.orientation.x = byte2Float(begin+16, begin+20);
//	imu.orientation.y = byte2Float(begin+20, begin+24);
//	imu.orientation.z = byte2Float(begin+24, begin+28);
//	imu.orientation.w = byte2Float(begin+28, begin+32);

	imu_pub.publish(imu);
}

void dataProcess(vector<unsigned char>& data)
{
	// cout << "data size: " << data.size() << endl;
	// for_each(data.begin(), data.end(), printE }); cout << endl;
	vector<unsigned char>::iterator position1, position2;
	position1 = find(data.begin(), data.end(), 0x55);
	position2 = find(position1+1, data.end(), 0x55);
	// printf("*position1: %x   *position2: %x \n", *position1, *position2);
	// cout << "distance: " << distance(position1, position2) << endl;
	if(distance(position1, position2) == 42)  //42
	{
		one_frame_data.assign(position1, position2);
		if(imu_mode == 0){
			dataParseRaw(one_frame_data);
		}
		else if(imu_mode == 1){
			dataParse(one_frame_data);
		}

	}

	if(position1 != data.end() || position2 != data.end())
		data.erase(data.begin(), position2);
	else
	{
		cout << "***********************" << endl;
		vector<unsigned char>().swap(data);
		cout << "data capacity: " << data.capacity() << "	" << "data size: " << data.size() << endl;
	}
	// cout << "data size: " << data.size() << endl;
	// for_each(data.begin(), data.end(), printE); cout << endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_serial");
	ros::NodeHandle nh;
	imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);


    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
    sp.set_option(serial_port::baud_rate(115200));   
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));

	unsigned char set_raw_data[] = {0x55, 0xb3, 0x00, 0x01};
	unsigned char set_process_data[] = {0x55, 0xb3, 0x00, 0x02};
	unsigned char set_start[] = {0x55, 0xb4, 0x00, 0x01};
	unsigned char set_stop[] = {0x55, 0xb4, 0x00, 0x00};



	if(imu_mode == 0)
	{
		for(int i = 0; i < sizeof(set_raw_data); ++i)
			printf("%x ", set_raw_data[i]);
		cout << endl;

		write(sp, buffer(set_raw_data, 4));
	}
	else if(imu_mode == 1){
		for(int i = 0; i < sizeof(set_process_data); ++i)
			printf("%x ", set_process_data[i]);
		cout << endl;

		write(sp, buffer(set_process_data, 4));
	}
	ros::Duration(1).sleep();
	write(sp, buffer(set_start, 4));


	ros::Rate loop_rate(100);
	data.clear();
	while(ros::ok())
	{
      	boost::system::error_code ignored_error;
		read_len = read(sp, buffer(buf), ignored_error);

		for(int i = 0; i < read_len; ++i)
			data.push_back(buf[i]);

		while(data.size() > 200){
//			 cout << "data size: " << data.size() << endl;
//			 for_each(data.begin(), data.end(), printE); cout << endl;
			dataProcess(data);
			cout << endl << "----------------------" << endl;
		}
        ros::spinOnce();
        loop_rate.sleep();
	}
}
