/*
 * zyf_imu_node.cpp
 *
 *  Created on: Jun 18, 2021
 *      Author: kychu
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>

#include <zyf.hpp>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv)
{
  ZYF_IMU *imu = nullptr;

  ros::init(argc, argv, "ZYF_IMU_DRV");
  ros::NodeHandle node;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  std::string port;
  if(ros::param::get("imu_dev", port)) {
    if(!port.empty()) {
      imu = new ZYF_IMU(node ,port);
      if(!imu->is_available()) {
        ROS_ERROR("ZYF IMU on port %s initialize failed, EXIT!", port.c_str());
        delete imu;
        imu = nullptr;
        return -3;
      }
    } else {
      ROS_ERROR("imu_dev was empty!");
      return -2;
    }
  } else {
    ROS_ERROR("NO parameter named 'imu_dev'!");
    return -1;
  }

  zyf_data_unit_t data;
  
  ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("/imu/raw_data", 50);
  geometry_msgs::Quaternion q;
  sensor_msgs::Imu imu_msg_data;
  imu_msg_data.header.frame_id = "/base_imu_link";
  imu_msg_data.linear_acceleration_covariance[0] = 0;
  imu_msg_data.linear_acceleration_covariance[4] = 0;
  imu_msg_data.linear_acceleration_covariance[8] = 0;

  imu_msg_data.angular_velocity_covariance[0] = 0;
  imu_msg_data.angular_velocity_covariance[4] = 0;
  imu_msg_data.angular_velocity_covariance[8] = 0;

  imu_msg_data.orientation_covariance[0] = 0;
  imu_msg_data.orientation_covariance[4] = 0;
  imu_msg_data.orientation_covariance[8] = 0;

  while(node.ok()) {
    if(imu->imu_data_take(data)) {
//      ROS_DEBUG("ax: %f, ay: %f, az: %f, gz: %f, yaw: %f, pitch: %f, roll: %f",
//                data.ax, data.ay, data.az, data.gz, data.yaw, data.pitch, data.roll);
      imu_msg_data.header.stamp = ros::Time::now();

      q = tf::createQuaternionMsgFromRollPitchYaw(data.roll, data.pitch, data.yaw);
      imu_msg_data.orientation.x = q.x;
      imu_msg_data.orientation.y = q.y;
      imu_msg_data.orientation.z = q.z;
      imu_msg_data.orientation.w = q.w;

      imu_msg_data.angular_velocity.x = 0;
      imu_msg_data.angular_velocity.y = 0;
      imu_msg_data.angular_velocity.z = data.gz;

      imu_msg_data.linear_acceleration.x = data.ax;
      imu_msg_data.linear_acceleration.y = data.ay;
      imu_msg_data.linear_acceleration.z = data.az;
      imu_pub.publish(imu_msg_data);
    }
    ros::spinOnce();
  }
  return 0;
}
