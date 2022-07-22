#include <iostream>
#include "calib.h"
void calib::load_images(std::vector<cv::Mat> &images) {
	
	cv::Size image_size;
	std::vector<cv::Point2f> image_points_buff;
	std::vector<std::vector<cv::Point2f>> image_points_seq;
	
	for (int i = 0; i < images.size(); i++) {
		cv::Mat image = images[i];
		if (i == 0) {
			// 第一幅图像
			image_size.width = image.cols;
			image_size.height = image.rows;
		}
		if (0 == cv::findChessboardCorners(image, board_size, image_points_buff,
		                                   cv::CALIB_CB_ADAPTIVE_THRESH |
		                                   cv::CALIB_CB_NORMALIZE_IMAGE /*+ cv::CALIB_CB_FAST_CHECK*/)) {
			// std::cout << "can not find chessboard corners! " << std::endl;
			// ofs << "can not find chessboard corners! " << std::endl;
			continue;
		} else {
			cv::cornerSubPix(image, image_points_buff, cv::Size(3, 3), cv::Size(-1, -1),
			                 cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
			// 对粗提取的角点精细化
			// cv::find4QuadCornerSubpix(image, image_points_buff, cv::Size(5, 5));
			// 保存亚像素角点
			image_points_seq.push_back(image_points_buff);
			// std::cout << "successed processing :" << num_img_successed_processing++ << std::endl;
			// cv::Mat image_color;
			// cv::cvtColor(image, image_color, cv::COLOR_GRAY2BGR);
			// cv::drawChessboardCorners(image_color, board_size, image_points_buff, true);
			// std::stringstream namestream;
			// std::string name;
			// namestream << "/" << i << "_corner.png";
			// namestream >> name;
			// cv::imwrite(path + name, image_color);
		}
	}
	
	// 根据检测到角点的图像的数量再次更新图像数目
	int num_of_images = image_points_seq.size();
	std::cout << "******************提取角点完成!******************" << std::endl;
	
	// 摄像机标定
	std::cout << "******************开始进行相机标定......******************" << std::endl;
	// 保存标定板上的角点的三维坐标
	std::vector<std::vector<cv::Point3f>> object_points;
	// 内外参
	// 内参
	cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	std::vector<int> point_counts;
	// 畸变系数: k1, k2, p1, p2, k3
	cv::Mat dist_coeff = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
	//平移向量
	std::vector<cv::Mat> t_vec;
	//旋转向量
	std::vector<cv::Mat> r_vec;
	// 初始化标定板上角点的三维坐标
	int i, j, k;
	for (k = 0; k < num_of_images; k++) {
		//std::cout << "image.NO:" << k << std::endl;
		std::vector<cv::Point3f> temp_point_set;
		for (i = 0; i < board_size.height; i++) {
			for (j = 0; j < board_size.width; j++) {
				cv::Point3f real_point;
				real_point.x = j * square_size.width;
				real_point.y = i * square_size.height;
				real_point.z = 0;
				// std::cout << "real_point cordinates" << real_point << std::endl;
				temp_point_set.push_back(real_point);
			}
		}
		object_points.push_back(temp_point_set);
	}
	//初始化每幅图像上的角点数量
	for (int i = 0; i < num_of_images; i++) {
		point_counts.push_back(board_size.width * board_size.height);
	}
	// 开始标定
	std::cout << "******************开始运行calibrateCamera!******************" << std::endl;
	cv::calibrateCamera(object_points, image_points_seq, image_size,
	                    camera_matrix, dist_coeff, r_vec, t_vec, 0);
	std::cout << "******************标定完成!******************" << std::endl;
	std::cout << camera_matrix << std::endl;
	std::cout << dist_coeff << std::endl;
	
	// 评价标定结果
	std::cout << "******************开始评定标定结果......******************" << std::endl;
	double total_err = 0.0;
	double err = 0.0;
	//保存重新计算得到的投影点
	std::vector<cv::Point2f> image_points2;
	std::cout << std::endl << "每幅图像的标定误差: " << std::endl;
	for (int i = 0; i < num_of_images; i++) {
		std::vector<cv::Point3f> temp_point_set = object_points[i];
		// 重映射
		cv::projectPoints(temp_point_set, r_vec[i], t_vec[i],
		                  camera_matrix, dist_coeff, image_points2);
		std::vector<cv::Point2f> temp_image_points = image_points_seq[i];
		cv::Mat temp_image_points_Mat = cv::Mat(1, temp_image_points.size(), CV_32FC2);
		cv::Mat temp_image_points2_Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < temp_image_points.size(); j++) {
			temp_image_points_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(temp_image_points[j].x, temp_image_points[j].y);
			temp_image_points2_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
		}
		err = cv::norm(temp_image_points_Mat, temp_image_points2_Mat, cv::NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "第" << i + 1 << "幅图像的平均误差: " << err << "像素" << std::endl;
	}
	
	std::cout << std::endl << "总体平均误差: " << total_err / num_of_images << "像素" << std::endl;
	std::cout << "******************评价完成!******************" << std::endl;
	
	// 输出标定结果
	std::cout << "******************标定结果如下:******************" << std::endl;
	std::cout << "相机内参:" << std::endl;
	std::cout << camera_matrix << std::endl;
	std::cout << "畸变系数:" << std::endl;
	std::cout << dist_coeff.t() << std::endl;
	// 输出到文件
}

calib::calib(uint8_t width,uint8_t height,uint8_t cell_x,uint8_t cell_y) {
	// 行列方向内角点数量
	board_size = cv::Size(width, height);
	square_size = cv::Size2f(cell_x, cell_y);
}
