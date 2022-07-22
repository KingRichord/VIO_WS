/*
 * zyf.cpp
 *
 *  Created on: Jun 18, 2021
 *      Author: kychu
 */

#include <zyf.hpp>
#include <cstdio>

ZYF_IMU::ZYF_IMU(ros::NodeHandle &n, std::string port) {
  this->node = n;
  this->port = port;
  try{
    this->serial.setPort(this->port); // select serial port
    this->serial.setBaudrate(115200); // configure the baud-rate to 115200
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // configure timeout to 1s
    this->serial.setTimeout(to);
    this->serial.open(); // open serial port
  } catch (serial::IOException &e)  {
    ROS_ERROR("Unable to open serial port: %s, %s", this->serial.getPort().c_str(), e.what());
    if(this->serial.isOpen()) this->serial.close();
    return;
  }
  if(this->serial.isOpen()) {
    this->serial.flush();
    ROS_INFO("Serial port: %s initialized and opened.", this->serial.getPort().c_str());
  }

  // reset decoder
  this->decoder.step = 0;
  this->decoder.rx_cnt = 0;

  this->stat_timer = this->node.createTimer(ros::Duration(1.0), &ZYF_IMU::stat_freq, this);
  this->rx_thread = boost::thread([this] { dataRec(); });

  this->available = true;
}

ZYF_IMU::~ZYF_IMU() {
  if(this->serial.isOpen()) {
    this->serial.close();
    ROS_INFO("Serial port: %s closed", this->serial.getPort().c_str());
  }
  if(this->stat_timer.isValid()) {
    this->stat_timer.stop();
  }
  this->rx_thread.join();
}

bool ZYF_IMU::imu_data_take(zyf_data_unit_t &d)
{
  if(this->data_avail.try_lock_for(std::chrono::milliseconds(5))) {
    d = this->unit_data;
    return true;
  } return false;
}

void ZYF_IMU::imu_update(zyf_frame_t &f)
{
  // combine bytes
  this->raw_data.ax = ((uint16_t)f.ax_h << 8) | f.ax_l;
  this->raw_data.ay = ((uint16_t)f.ay_h << 8) | f.ay_l;
  this->raw_data.az = ((uint16_t)f.az_h << 8) | f.az_l;
  this->raw_data.gz = ((uint16_t)f.gz_h << 8) | f.gz_l;
  this->raw_data.yaw = ((uint16_t)f.yaw_h << 8) | f.yaw_l;
  this->raw_data.pitch = ((uint16_t)f.pitch_h << 8) | f.pitch_l;
  this->raw_data.roll = ((uint16_t)f.roll_h << 8) | f.roll_l;
  // compute unit data
  this->unit_data.ax = this->raw_data.ax;
  this->unit_data.ay = this->raw_data.ay;
  this->unit_data.az = this->raw_data.az;
  this->unit_data.gz = ((float)this->raw_data.gz / 100.0) * 0.01745329251994329576923690768;
  this->unit_data.yaw = ((float)this->raw_data.yaw / 100.0) * 0.01745329251994329576923690768;
  this->unit_data.pitch = ((float)this->raw_data.pitch / 100.0) * 0.01745329251994329576923690768;
  this->unit_data.roll = ((float)this->raw_data.roll / 100.0) * 0.01745329251994329576923690768;
  this->data_avail.unlock();
}

void ZYF_IMU::dataRec(void)
{
  ros::Rate rate(200);
  std::string recv;
  while(this->node.ok()) {
    if(this->serial.isOpen()) {
      if(this->serial.available()) {
        recv = this->serial.read(this->serial.available());
        for(char &i : recv) {
          this->imu_decode((uint8_t)i);
        }
      }
    }
    rate.sleep();
  }
}

void ZYF_IMU::stat_freq(const ros::TimerEvent&)
{
  this->frame_rate = this->frame_count;
  this->frame_count = 0;
  ROS_DEBUG("IMU data frame rate: %d", this->frame_rate);
}

bool ZYF_IMU::frame_checksum(zyf_frame_t &f)
{
  uint16_t yaw_rate = ((uint16_t)f.gz_h << 8) | f.gz_l;
  uint16_t yaw = ((uint16_t)f.yaw_h << 8) | f.yaw_l;
  uint16_t pitch = ((uint16_t)f.pitch_h << 8) | f.pitch_l;
  uint16_t roll = ((uint16_t)f.roll_h << 8) | f.roll_l;
  uint16_t chksum = ((uint16_t)f.chk_h << 8) | f.chk_l;
  if(chksum == ((yaw_rate + yaw + pitch + roll) & 0xFFFF)) return true;
  return false;
}

void ZYF_IMU::imu_decode(uint8_t data)
{
  switch(this->decoder.step) {
    case 0:
      if(data == 0xA5) {
        this->decoder.step ++;
        this->decoder.frame.stx1 = data;
      }
    break;
    case 1:
      if(data == 0xA5) {
        this->decoder.step ++;
        this->decoder.rx_cnt = 0;
        this->decoder.frame.stx2 = data;
      } else {
        this->decoder.step = 0;
      }
    break;
    case 2:
      this->decoder.frame.buff[2 + this->decoder.rx_cnt ++] = data;
      if(this->decoder.rx_cnt == 16) {
        if(this->frame_checksum(this->decoder.frame)) {
          this->frame_count ++;
          this->imu_update(this->decoder.frame);
        } else {
          ROS_WARN("ZYF IMU frame checksum failed!");
        }
        this->decoder.step = 0;
      }
    break;
    default: this->decoder.step = 0; break;
  }
}
