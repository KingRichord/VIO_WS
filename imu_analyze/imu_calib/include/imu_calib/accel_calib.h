/**
 * \file accel_calib.h
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Class for calculating and applying accelerometer calibration parameters
 */

#include <Eigen/Dense>

#include <string>

namespace imu_calib
{

class Calib
{
public:

  Calib();

  // status
  bool calibReady();

  // file I/O
  bool loadCalib_from_yaml(std::string calib_file);
  

  // calibration application
  void applyCalib(double acc_raw_x, double acc_raw_y, double acc_raw_z, double *acc_corr_x, double *acc_corr_y, double *acc_corr_z,
		          double gyro_raw_x, double gyro_raw_y, double gyro_raw_z, double *gyro_corr_x, double *gyro_corr_y, double *gyro_corr_z);

protected:
  bool calib_ready_;
  
    Eigen::Matrix3d ACC_M_;   // 加速度正交误差
    Eigen::Matrix3d ACC_S_;   // 加速度尺度误差
    Eigen::Vector3d ACC_BIAS_;  //加速度偏置
	
    Eigen::Matrix3d GYRO_M_;   // 陀螺仪正交误差
    Eigen::Matrix3d GYRO_S_;   // 陀螺仪尺度误差
    Eigen::Vector3d GYRO_BIAS_;  //陀螺仪偏置

};

} // namespace accel_calib
