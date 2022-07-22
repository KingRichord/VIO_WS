/**
 * \file apply_calib.cpp
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for applying a previously computed calibration to IMU data
 */

#include "imu_calib/apply_calib.h"

namespace imu_calib
{

ApplyCalib::ApplyCalib()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string calib_file;
  nh_private.param<std::string>("calib_file", calib_file, "imu_calib.yaml");
  ROS_INFO("start apply calib !!!");
  if (!calib_.loadCalib_from_yaml(calib_file) || !calib_.calibReady())
  {
    ROS_FATAL("Calibration could not be loaded");
    ros::shutdown();
  }

  int queue_size;
  nh_private.param<int>("queue_size", queue_size, 5);

  raw_sub_ = nh.subscribe("/camera/imu", queue_size, &ApplyCalib::rawImuCallback, this);
  corrected_pub_ = nh.advertise<sensor_msgs::Imu>("/camera/correct_imu", queue_size);
}

void ApplyCalib::rawImuCallback(sensor_msgs::Imu::ConstPtr raw)
{
  sensor_msgs::Imu corrected = *raw;
  calib_.applyCalib(raw->linear_acceleration.x, raw->linear_acceleration.y, raw->linear_acceleration.z,
                    &corrected.linear_acceleration.x, &corrected.linear_acceleration.y, &corrected.linear_acceleration.z,
		            raw->angular_velocity.x, raw->angular_velocity.y, raw->angular_velocity.z,
		            &corrected.angular_velocity.x, &corrected.angular_velocity.y, &corrected.angular_velocity.z
		            );
  corrected_pub_.publish(corrected);
}

} // namespace accel_calib
