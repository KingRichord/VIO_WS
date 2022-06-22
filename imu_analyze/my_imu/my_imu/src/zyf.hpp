/*
 * zyf.hpp
 *
 *  Created on: Jun 18, 2021
 *      Author: kychu
 */

#pragma once

#include <ros/ros.h>
#include "../serial/include/serial/serial.h"

#include <mutex>

#include <boost/thread.hpp>
typedef struct {
  double ax, ay, az, gx, gy, gz, tmp;
} pico_imu_data;

class ZYF_IMU {
public:
  ZYF_IMU(ros::NodeHandle &n, std::string port);
  ~ZYF_IMU();
  bool is_available(void) { return this->available; }
  bool imu_data_take(pico_imu_data &d);
private:
  std::string port;
  serial::Serial serial;
  ros::NodeHandle node;
  bool available{false};
    pico_imu_data imu_data;


  std::timed_mutex data_avail;

  int frame_rate{0};
  int frame_count{0};

  ros::Timer stat_timer;
  void stat_freq(const ros::TimerEvent&);
  void imu_update(std::vector<double> &f);
  boost::thread rx_thread;
  void dataRec(void);
};
