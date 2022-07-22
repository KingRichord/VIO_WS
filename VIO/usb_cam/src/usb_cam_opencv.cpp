#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>  // 将ROS下的sensor_msgs/Image消息类型转化为cv::Mat数据类型
#include <sensor_msgs/image_encodings.h> // ROS下对图像进行处理
#include <image_transport/image_transport.h> // 用来发布和订阅图像信息

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <camera_info_manager/camera_info_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imageGet_node");  // ros初始化，定义节点名为imageGet_node
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);  //  类似ROS句柄
    
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    std::string image_topic;
    nh.param("image_topic",image_topic,std::string("/raw_image"));
    image_transport::CameraPublisher image_pub_ =it.advertiseCamera(image_topic, 1);

    sensor_msgs::CameraInfo cinf;
    image_transport::Publisher image_only_ = it.advertise(image_topic,1);

    int rate;
    nh.param("publish_rate",rate,10);

    ros::Rate loop_rate(rate);   // 设置刷新频率，Hz

    std::string device_name,frame_id;
    bool image_show=false;
    nh.param("camera_frame_id",frame_id,std::string("/base_link_usb_cam"));
    nh.param("video_device",device_name,std::string("/dev/usb_cam"));
    nh.param("image_show",image_show,false);

    std::string camera_name_,camera_info_url_;
    bool release_;
    nh.param("camera_name", camera_name_, std::string("head_camera"));
    nh.param("camera_info_url", camera_info_url_, std::string(""));
    nh.param("release", release_, false);


    std::cout<<"camera_info_url:"<<camera_info_url_<<std::endl;


    if(release_)
    {
        cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, camera_name_, camera_info_url_));

        if(!cinfo_->isCalibrated())
        {
            ROS_ERROR("camera not calibrated return ");
            exit(0);
        }
    }



    std::cout<<"rate"<<rate<<std::endl;
    cv::Mat imageRaw;  // 原始图像保存
    cv::VideoCapture capture(device_name);   // 创建摄像头捕获，并打开摄像头0(一般是0,2....)

    while(ros::ok())
    {
        if(capture.isOpened() == 0) 
        {
            ROS_WARN("waiting camera");
            capture.open(device_name);
        }
        else
        {
            break;
        }
        
    }
    if(release_){

        cinf  = cinfo_->getCameraInfo();
        cinf.header.frame_id=frame_id;
        std::cout<<"cinf height"<<cinf.height<<"width "<<cinf.width<<std::endl;

    }

    while(nh.ok())
    {
        loop_rate.sleep();              // 照应上面设置的频率
        if(!capture.read(imageRaw))
        {
            ROS_WARN("waiting camera");
            capture.open(device_name);
            continue;
        }
        if(image_show)
        {
            cv::imshow("veiwer", imageRaw);  // 将图像输出到窗口
        }
        sensor_msgs::ImagePtr  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageRaw).toImageMsg();  // 图像格式转换
        msg->header.frame_id=frame_id;
        msg->header.stamp = ros::Time::now();

        if(release_)
        {
            msg->width = cinf.width;
            msg->height = cinf.height;
            cinf.header.stamp = ros::Time::now();
            image_pub_.publish(*msg, cinf);
        }
        else
        {
            image_only_.publish(msg);
        }


        //image_pub.publish(msg);         // 发布图像信息
    }
    capture.release();
    ROS_INFO("shut down");
}