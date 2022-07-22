#ifndef BENCHMARK_PUBLISHER_CALIB_H
#define BENCHMARK_PUBLISHER_CALIB_H
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Dense>
class calib {
public:
	calib();
	calib(uint8_t x,uint8_t y,uint8_t cell_x,uint8_t cell_y);
	~calib();
	void load_images(std::vector<cv::Mat> &pic);
private:
	// 标定板中点的分布
	cv::Size board_size;
	// 标定板棋盘格实际尺寸(单位要与pose.txt中机器人位置的单位一致) mm
	cv::Size2f square_size;
	// 图像的数量
	int num_of_all_images;
};


#endif //BENCHMARK_PUBLISHER_CALIB_H
