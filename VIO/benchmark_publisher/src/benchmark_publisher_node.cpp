#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include <queue>

#define CVUI_IMPLEMENTATION
#include "cvui.h"
#define WINDOW1_NAME "data"

using namespace std;
using namespace Eigen;

uint32_t  num  =0;

std::mutex m_odom_buf,m_optitrack_buf;
queue<nav_msgs::Odometry::ConstPtr> odom_buf;
queue<geometry_msgs::PoseStamped::ConstPtr> optitrack_pose_buf;
std::vector<cv::Mat> pic_need_to_write;
std::vector<Eigen::Isometry3d> pose_need_to_write;
void OdomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
	// m_odom_buf.lock();
	// odom_buf.push(odom_msg);
	// m_odom_buf.unlock();
}

void OptitrackerCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
	m_optitrack_buf.lock();
	// ROS_WARN("new Optitracker data");
	optitrack_pose_buf.push(msg);
	m_optitrack_buf.unlock();
}

// Process the image and trigger the estimator.
void ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
	// 同步参考https://github.com/softdream/imu_encoder_fusion
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
	
	ROS_INFO("cam:%f",msg->header.stamp.toSec());
	
	// 这里进行软件时间同步
	//  设置标记位
	if (optitrack_pose_buf.empty()) {
		ROS_WARN("no optitrack data");
		return;
	}
	// 如果optitrack最新的数据比当前的tag消息还要老 （最新的odom略晚于TAG时间）
	if ((optitrack_pose_buf.back()->header.stamp.toSec() < msg->header.stamp.toSec())) {
		// std::cout<<"optitrack太晚了"<<std::endl;
		// optitrack 最新一帧数据比当前的tag数据还要老 ：注意:这里如果optitrack数据太新，就没有用
		while (!optitrack_pose_buf.empty()) {
			optitrack_pose_buf.pop();
		}  //清空当前的optitrack数据
		ROS_WARN("clear1 optitrack data");
		return;
	}
	// 如果optitrack最老的数据比当前的TAG数据还要新  （跳过当前帧）
	if ((optitrack_pose_buf.front()->header.stamp.toSec() > msg->header.stamp.toSec())) {
		// std::cout<<"跳过当前TAG"<<std::endl;
		ROS_WARN("clear 2 optiitrack data");
		return;
	}
	// 第一步：得到位姿对应的optitrack数据
	// 利用时间权重合成tag对应的optitrack数据
	geometry_msgs::PoseStampedConstPtr pre;   //收到TAG数据时上一最近邻时刻的odom数据
	geometry_msgs::PoseStampedConstPtr last;  // 收到TAG数据时下一最近邻时刻的odom数据
	/*
	*  目前optitrack的频率是20HZ，两帧IMU之间的差异为0.05，在这里计算的是对的
	   pre time optitrack data : 1626162825.604743
	   TAG time:            1626162825.634622
	   last time optitrack data :1626162825.654796
	*/
	while (optitrack_pose_buf.front()->header.stamp.toSec() < (msg->header.stamp.toSec())) {
		pre = optitrack_pose_buf.front();
		if (!optitrack_pose_buf.empty()) optitrack_pose_buf.pop();
	}
	last = optitrack_pose_buf.front();
	//---------t_last-----------t------------t_prev------------->时间流失变大
	//             |      w1     |      w2       |
	double w1 = msg->header.stamp.toSec() - last->header.stamp.toSec();
	double w2 = pre->header.stamp.toSec() - msg->header.stamp.toSec();
	double dx = pre->pose.position.x - last->pose.position.x;
	double dy = pre->pose.position.y - last->pose.position.y;
	double dz = pre->pose.position.z - last->pose.position.z;
	// ROS_ASSERT(w1 < 0);
	// ROS_ASSERT(w2 < 0);
	Eigen::Vector3d new_p
	                  {last->pose.position.x + dx * w1 / (w1 + w2),
	                   last->pose.position.y + dy * w1 / (w1 + w2),
	                   last->pose.position.z + dz * w1 / (w1 + w2)};
	Eigen::Quaterniond pre_q
						{pre->pose.orientation.w, pre->pose.orientation.x, pre->pose.orientation.y,
	                      pre->pose.orientation.z};
	Eigen::Quaterniond last_q{last->pose.orientation.w, last->pose.orientation.x, last->pose.orientation.y,
	                       last->pose.orientation.z};
	Eigen::Quaterniond new_q = last_q.slerp(w1 / (w1 + w2),pre_q); new_q.normalized();
	// T_odomBase
	//tf2::Transform T_odomBase = tf2::Transform(last_q, new_p);
	ROS_INFO("pre:%f",pre->header.stamp.toSec());
	ROS_INFO("las:%f",last->header.stamp.toSec());
	cv::Mat img = cv_ptr->image;
	cv::Mat show_img = img.clone();
	// cv::imshow("this",show_img);
	//cvui::text(show_img, 0, 0, "Hello world!");
	if (cvui::button(show_img, show_img.cols/2, show_img.rows/2, "SAVE")) {
		ROS_INFO("========================================================");
		ROS_INFO("push");
		num ++;
		Eigen::Isometry3d T_marker_world; T_marker_world.setIdentity();
		T_marker_world.pretranslate(new_p);
		T_marker_world.rotate(new_q.toRotationMatrix());
		pose_need_to_write.emplace_back(T_marker_world);
		pic_need_to_write.emplace_back(img);
	}
	if (cvui::button(show_img, show_img.cols/2 +100, show_img.rows/2, "CALIB")) {
		ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
		 //将位置信息进行保存
		std::string package_path = ros::package::getPath("benchmark_publisher");
		std::string img_path = package_path + "/data/" + std::to_string(num++)+ ".png";
		std::string pose_path = package_path + "/data/pose.txt";
		ofstream ofs(pose_path );
		for (int i = 0; i < num -1; ++i) {
			ofs << i <<" ";
			ofs <<pose_need_to_write[i].translation().x()<<" "
			    <<pose_need_to_write[i].translation().y()<<" "
				<<pose_need_to_write[i].translation().z()<<" ";
			Eigen::Quaterniond q(pose_need_to_write[i].rotation());
			ofs << q.x()<<" "<< q.y()<<" "<< q.z()<<" "<< q.w();
			ofs <<std::endl;
		}
		ofs.close();
		// for (int i = 0; i < num; ++i) {
		// 	cv::imwrite(img_path, img);
		// }
	}
	cvui::imshow(WINDOW1_NAME, show_img);
	cv::waitKey(1);
}

int main(int argc, char **argv) {
	cvui::init(WINDOW1_NAME);
	ros::init(argc, argv, "benchmark_publisher");
	ros::NodeHandle n("~");
	// 订阅图像信息
	ros::Subscriber sub_img = n.subscribe("/usb_cam/image_raw", 1, &ImageCallback);
	
	// 发布里程计信息
	ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("odometry", 1000);
	ros::Publisher pub_path = n.advertise<nav_msgs::Path>("path", 1000);
	// 订阅VIO的里程计数据
	ros::Subscriber sub_odom = n.subscribe("vio_topic_name", 1000, OdomCallback);
	// 订阅光追踪位姿数据
	ros::Subscriber sub_optitracker = n.subscribe("/vrpn_client_node/Rigid0/pose", 100, OptitrackerCallback);
	// 订阅图像数据,实时解算位姿数据
	
	ros::Rate r(20);
	ros::spin();
}
 // 二维码相机和Optitrack的标定类似与eye in hand 方式的标定算法。
 