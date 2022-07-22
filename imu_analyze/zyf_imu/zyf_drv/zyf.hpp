/*
 * zyf.hpp
 *
 *  Created on: Jun 18, 2021
 *      Author: kychu
 */

#pragma once

#include <ros/ros.h>
#include <serial/serial.h>

#include <mutex>

#include <boost/thread.hpp>

typedef union {
  struct {
    uint8_t stx1;
    uint8_t stx2;
    uint8_t ax_l;
    uint8_t ax_h;
    uint8_t ay_l;
    uint8_t ay_h;
    uint8_t az_l;
    uint8_t az_h;
    uint8_t gz_l;
    uint8_t gz_h;
    uint8_t yaw_l;
    uint8_t yaw_h;
    uint8_t pitch_l;
    uint8_t pitch_h;
    uint8_t roll_l;
    uint8_t roll_h;
    uint8_t chk_l;
    uint8_t chk_h;
  };
  uint8_t buff[18];
} zyf_frame_t;

typedef struct {
  zyf_frame_t frame;
  uint8_t rx_cnt;
  uint8_t step;
} zyf_decode_t;

typedef struct {
  int16_t ax, ay, az, gz, yaw, pitch, roll;
} zyf_data_raw_t;

typedef struct {
  float ax, ay, az, gz, yaw, pitch, roll;
} zyf_data_unit_t;

class ZYF_IMU {
public:
  ZYF_IMU(ros::NodeHandle &n, std::string port);
  ~ZYF_IMU();
  bool is_available(void) { return this->available; }
  bool imu_data_take(zyf_data_unit_t &d);
private:
  std::string port;
  serial::Serial serial;
  ros::NodeHandle node;
  bool available{false};
  zyf_data_raw_t raw_data;
  zyf_data_unit_t unit_data;

  std::timed_mutex data_avail;

  int frame_rate{0};
  int frame_count{0};

  ros::Timer stat_timer;
  void stat_freq(const ros::TimerEvent&);

  boost::thread rx_thread;
  void dataRec(void);
  void imu_update(zyf_frame_t &f);

  zyf_decode_t decoder;
  void imu_decode(uint8_t data);
  bool frame_checksum(zyf_frame_t &f);
};
