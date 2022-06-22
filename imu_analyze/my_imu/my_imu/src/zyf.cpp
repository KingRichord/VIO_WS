/*
 * zyf.cpp
 *
 *  Created on: Jun 18, 2021
 *      Author: kychu
 */

#include "zyf.hpp"
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

bool ZYF_IMU::imu_data_take(pico_imu_data &d)
{
  if(this->data_avail.try_lock_for(std::chrono::milliseconds(5))) {
    d = this->imu_data;
    return true;
  } return false;
}

void ZYF_IMU::imu_update(std::vector<double> &f)
{
  // combine bytes
    this->imu_data.ax = f[0];
    this->imu_data.ay = f[1];
    this->imu_data.az = f[2];

    this->imu_data.gx = f[3];
    this->imu_data.gy = f[4];
    this->imu_data.gz = f[5];

    this->imu_data.tmp = f[6];
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
        if(!recv.empty() && recv.size() >= 60)
        {
            // std::cout <<"<recv : "<<recv.size()<<std::endl;
            std::vector<double> data;
            std::stringstream sstr(recv);
            std::string token;
            while(getline(sstr, token, ','))
            {
                data.push_back(stod(token));
            }
            // std::cout <<"<data.size : "<<data.size()<<std::endl;
            if(data.size() != 7 ) continue;
            imu_update(data);
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

