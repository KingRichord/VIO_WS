/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>

namespace usb_cam {

class UsbCamNode {
public:
    // private ROS node handle
    ros::NodeHandle node_;

    // shared image message
    sensor_msgs::Image img_;
    image_transport::CameraPublisher image_pub_;

    // parameters
    std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
    //std::string start_service_name_, start_service_name_;
    bool streaming_status_;
    int image_width_, image_height_, framerate_, brightness_, contrast_, saturation_, hue_, 
        white_balance_temperature_auto_, gamma_, gain_, power_line_frequency_, white_balance_temperature_,
        sharpness_, backlight_compensation_, exposure_auto_, exposure_absolute_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    UsbCam cam_;

    ros::ServiceServer service_start_, service_stop_;


    bool service_start_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        cam_.start_capturing();
        return true;
    }


    bool service_stop_cap( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        cam_.stop_capturing();
        return true;
    }

    UsbCamNode(): node_("~") {
        // advertise the main image topic
        image_transport::ImageTransport it(node_);
        image_pub_ = it.advertiseCamera("image_raw", 1);

        // grab the parameters
        node_.param("video_device", video_device_name_, std::string("/dev/video0"));
        // brightness 0x00980900 (int): min=-64 max=64 step=1 default=0
        node_.param("brightness", brightness_, 0);
        // contrast 0x00980901 (int): min=0 max=100 step=1 default=40
        node_.param("contrast", contrast_, 40);
        // saturation 0x00980902 (int): min=0 max=100 step=1 default=64
        node_.param("saturation", saturation_, 64);
        // hue 0x00980903 (int): min=-180 max=180 step=1 default=0
        node_.param("hue", hue_, 0);
        // white_balance_temperature_auto 0x0098090c (bool): default=1
        // 0: false (manual)
        // 1: true (auto)
        node_.param("white_balance_temperature_auto", white_balance_temperature_auto_, 1);
        // gamma 0x00980910 (int): min=100 max=500 step=1 default=300
        node_.param("gamma", gamma_, 300);
        // gain 0x00980913 (int): min=1 max=128 step=1 default=64
        node_.param("gain", gain_, 64);
        // power_line_frequency 0x00980918 (menu): min=0 max=2 default=1
        node_.param("power_line_frequency", power_line_frequency_, 1);
        // white_balance_temperature 0x0098091a (int): min=2800 max=6500 step=10 default=4600
        node_.param("white_balance_temperature", white_balance_temperature_, 4600);
        // sharpness 0x0098091b (int): min=0 max=100 step=1 default=50
        node_.param("sharpness", sharpness_, 50);
        // backlight_compensation 0x0098091c (int): min=0 max=2 step=1 default=0
        node_.param("backlight_compensation",  backlight_compensation_, 0);
        // exposure_auto 0x009a0901 (menu): min=0 max=3 default=3
        /*
            V4L2_CID_EXPOSURE_AUTO: enum v4l2_exposure_auto_type
            Enables automatic adjustments of the exposure time and/or iris aperture.
            The effect of manual changes of the exposure time or iris aperture while
            these features are enabled is undefined, drivers should ignore such requests.
            Possible values are:
            V4L2_EXPOSURE_AUTO - Automatic exposure time, automatic iris aperture.
            V4L2_EXPOSURE_MANUAL - Manual exposure time, manual iris.
            V4L2_EXPOSURE_SHUTTER_PRIORITY - Manual exposure time, auto iris.
            V4L2_EXPOSURE_APERTURE_PRIORITY - Auto exposure time, manual iris.
        */
        node_.param("exposure_auto", exposure_auto_, 3);
        // exposure_absolute 0x009a0902 (int): min=2 max=10000 step=1 default=6 flags=inactive
        node_.param("exposure_absolute", exposure_absolute_, 6);
   
        // possible values: mmap, read, userptr
        node_.param("io_method", io_method_name_, std::string("mmap"));
        node_.param("image_width", image_width_, 640);
        node_.param("image_height", image_height_, 480);
        node_.param("framerate", framerate_, 30);
        // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
        node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
        
        // load the camera info
        node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
        node_.param("camera_name", camera_name_, std::string("head_camera"));
        node_.param("camera_info_url", camera_info_url_, std::string(""));
        cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

        // create Services
        service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
        service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

        // check for default camera info
        if (!cinfo_->isCalibrated()) {
            cinfo_->setCameraName(video_device_name_);
            sensor_msgs::CameraInfo camera_info;
            camera_info.header.frame_id = img_.header.frame_id;
            camera_info.width = image_width_;
            camera_info.height = image_height_;
            cinfo_->setCameraInfo(camera_info);
        }

        ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
            image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

        // set the IO method
        UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
        if(io_method == UsbCam::IO_METHOD_UNKNOWN) {
            ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
            node_.shutdown();
            return;
        }

        // set the pixel format
        UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
        if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN) {
            ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
            node_.shutdown();
            return;
        }

        // start the camera
        cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
                image_height_, framerate_);

        // set camera parameters

        // brightness 0x00980900 (int): min=-64 max=64 step=1 default=0
        if (-64 <= brightness_ && brightness_ <= 64) {
            cam_.set_v4l_parameter("brightness", brightness_);
        }

        // contrast 0x00980901 (int): min=0 max=100 step=1 default=40
        if (0 <= contrast_ && contrast_ <= 100) {
            cam_.set_v4l_parameter("contrast", contrast_);
        }

        // saturation 0x00980902 (int): min=0 max=100 step=1 default=64
        if (0 <= saturation_ && saturation_ <= 100) {
            cam_.set_v4l_parameter("saturation", saturation_);
        }

        // hue 0x00980903 (int): min=-180 max=180 step=1 default=0
        if (-180 <= hue_ && hue_ <= 180) {
            cam_.set_v4l_parameter("hue", hue_);
        }

        // white_balance_temperature_auto 0x0098090c (bool): default=1
        // white_balance_temperature 0x0098091a (int): min=2800 max=6500 step=10 default=4600
        if (0 <= white_balance_temperature_auto_ && white_balance_temperature_auto_ <= 1) {
            cam_.set_v4l_parameter("white_balance_temperature_auto", white_balance_temperature_auto_);
            if (white_balance_temperature_auto_ == 0) {
                cam_.set_v4l_parameter("white_balance_temperature", white_balance_temperature_);
            }
        }

        // gamma 0x00980910 (int): min=100 max=500 step=1 default=300
        if (100 <= gamma_ && gamma_ <= 500) {
            cam_.set_v4l_parameter("gamma", gamma_);
        }

        // gain 0x00980913 (int): min=1 max=128 step=1 default=64
        if (1 <= gain_ && gain_ <= 128) {
            cam_.set_v4l_parameter("gain", gain_);
        }

        // power_line_frequency 0x00980918 (menu): min=0 max=2 default=1
        if (0 <= power_line_frequency_ && power_line_frequency_ <= 2) {
            cam_.set_v4l_parameter("power_line_frequency", power_line_frequency_);
        }

        // sharpness 0x0098091b (int): min=0 max=100 step=1 default=50
        if (0 <= sharpness_ && sharpness_ <= 100) {
            cam_.set_v4l_parameter("sharpness", sharpness_);
        }

        // backlight_compensation 0x0098091c (int): min=0 max=2 step=1 default=0
        if (0 <= backlight_compensation_ && backlight_compensation_ <= 2) {
            cam_.set_v4l_parameter("backlight_compensation", backlight_compensation_);
        }
       
        // exposure_auto 0x009a0901 (menu): min=0 max=3 default=3
        // exposure_absolute 0x009a0902 (int): min=2 max=10000 step=1 default=6 flags=inactive
        /*
            V4L2_CID_EXPOSURE_AUTO: enum v4l2_exposure_auto_type
            Enables automatic adjustments of the exposure time and/or iris aperture.
            The effect of manual changes of the exposure time or iris aperture while
            these features are enabled is undefined, drivers should ignore such requests.
            Possible values are:
            V4L2_EXPOSURE_AUTO - Automatic exposure time, automatic iris aperture.
            V4L2_EXPOSURE_MANUAL - Manual exposure time, manual iris.
            V4L2_EXPOSURE_SHUTTER_PRIORITY - Manual exposure time, auto iris.
            V4L2_EXPOSURE_APERTURE_PRIORITY - Auto exposure time, manual iris.
        */
        if (0 <= exposure_auto_ && exposure_auto_ <= 3) {
            cam_.set_v4l_parameter("exposure_auto", exposure_auto_);
            if (exposure_auto_ == 1 || exposure_auto_ == 2) {
                cam_.set_v4l_parameter("exposure_absolute", exposure_absolute_);
            }
        }
    }


    virtual ~UsbCamNode() {
        cam_.shutdown();
    }


    bool take_and_send_image() {
        // grab the image
        cam_.grab_image(&img_);

        // grab the camera info
        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
        ci->header.frame_id = img_.header.frame_id;
        ci->header.stamp = img_.header.stamp;

        // publish the image
        image_pub_.publish(img_, *ci);

        return true;
    }


    bool spin()
    {
        ros::Rate loop_rate(this->framerate_);
        while (node_.ok()) {
            if (cam_.is_capturing()) {
                if (!take_and_send_image()) {
                    ROS_WARN("USB camera did not respond in time.");
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        return true;
    }
};
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "usb_cam");
    usb_cam::UsbCamNode a;
    a.spin();
    return EXIT_SUCCESS;
}
