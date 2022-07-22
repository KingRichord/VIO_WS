
/**
 * \file accel_calib.cpp
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Class for calculating and applying accelerometer calibration parameters
 */

#include "imu_calib/accel_calib.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

namespace imu_calib
{

Calib::Calib() :
  calib_ready_(false){}


bool Calib::calibReady()
{
  return calib_ready_;
}

void Calib::applyCalib(double acc_raw_x, double acc_raw_y, double acc_raw_z, double *acc_corr_x, double *acc_corr_y, double *acc_corr_z,
	                   double gyro_raw_x, double gyro_raw_y, double gyro_raw_z, double *gyro_corr_x, double *gyro_corr_y, double *gyro_corr_z)
{
  Eigen::Vector3d raw_accel(acc_raw_x, acc_raw_y, acc_raw_z);
  Eigen::Vector3d result = ACC_M_*ACC_S_*(raw_accel -ACC_BIAS_);
	*acc_corr_x = result(0);
	*acc_corr_y = result(1);
	*acc_corr_z = result(2);

  
  
  Eigen::Vector3d raw_gyro(gyro_raw_x, gyro_raw_y, gyro_raw_z);
  result = GYRO_M_*GYRO_S_*(raw_gyro -GYRO_BIAS_);
  
  if(abs(result(0)) < 0.006 ) *gyro_corr_x  = 0.;
  else *gyro_corr_x = result(0);
  
  if(abs(result(1)) < 0.006 ) *gyro_corr_y  = 0.;
  else *gyro_corr_y = result(0);
  
  if(abs(result(2)) < 0.006 ) *gyro_corr_z  = 0.;
  else *gyro_corr_z = result(0);
}

bool Calib::loadCalib_from_yaml(std::string calib_file) {

    try
    {
        YAML::Node node = YAML::LoadFile(calib_file);

        assert(node["ACC_M"].IsSequence() && node["ACC_M"].size() == 9);
        assert(node["ACC_S"].IsSequence() && node["ACC_S"].size() == 9);
        assert(node["ACC_bias"].IsSequence() && node["ACC_bias"].size() == 3);

        assert(node["GYRO_M"].IsSequence() && node["GYRO_M"].size() == 9);
        assert(node["GYRO_S"].IsSequence() && node["GYRO_S"].size() == 9);
        assert(node["GYRO_bias"].IsSequence() && node["GYRO_bias"].size() == 3);

        // 加载加速度参数
        for (int i = 0; i < 9; i++)
        {
            ACC_M_(i/3,i%3) = node["ACC_M"][i].as<double>();
			ACC_S_(i/3,i%3) = node["ACC_S"][i].as<double>();
        }
        for (int i = 0; i < 3; i++)
        {
            ACC_BIAS_(i) = node["ACC_bias"][i].as<double>();
        }
        // 加载陀螺仪参数
        for (int i = 0; i < 9; i++)
        {
            GYRO_M_(i/3,i%3) = node["GYRO_M"][i].as<double>();
			GYRO_S_(i/3,i%3) = node["GYRO_S"][i].as<double>();
        }
        for (int i = 0; i < 3; i++)
        {
            GYRO_BIAS_(i) = node["GYRO_bias"][i].as<double>();
        }
        calib_ready_ = true;
        return true;
    }
    catch (...)
    {
        return false;
    }
    return false;
}

} // namespace accel_calib
