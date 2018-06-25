//
// Created by songjin on 18-5-22.
//
#include "detection/DetectionByColor.h"
#include <opencv2/opencv.hpp>
using namespace cv;

float GetCross(Point2f& p1, Point2f& p2, Point2f& p)
{
	return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}
// 判断第二大区域是否在最大区域内部
bool IsInReigon(cv::RotatedRect &roi_1, cv::RotatedRect &roi_2)
{
	// 将RotatedRect边缘点赋值给 Point2f
	cv::Point2f Q[4];
	roi_1.points(Q);

	cv::Point2f P[4];
	roi_2.points(P);

	// 判断第二大区域边缘每个点是否在最大区域内部
	bool IsPointInMatrix_0 = GetCross(Q[0], Q[1], P[0]) * GetCross(Q[2], Q[3], P[0]) >= 0 && GetCross(Q[1], Q[2], P[0]) * GetCross(Q[3], Q[0], P[0]) >= 0;
	bool IsPointInMatrix_1 = GetCross(Q[0], Q[1], P[1]) * GetCross(Q[2], Q[3], P[1]) >= 0 && GetCross(Q[1], Q[2], P[1]) * GetCross(Q[3], Q[0], P[1]) >= 0;
	bool IsPointInMatrix_2 = GetCross(Q[0], Q[1], P[2]) * GetCross(Q[2], Q[3], P[2]) >= 0 && GetCross(Q[1], Q[2], P[2]) * GetCross(Q[3], Q[0], P[2]) >= 0;
	bool IsPointInMatrix_3 = GetCross(Q[0], Q[1], P[3]) * GetCross(Q[2], Q[3], P[3]) >= 0 && GetCross(Q[1], Q[2], P[3]) * GetCross(Q[3], Q[0], P[3]) >= 0;

	if (IsPointInMatrix_0 && IsPointInMatrix_1 && IsPointInMatrix_2 && IsPointInMatrix_3)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool isSmaller(const std::vector<cv::Point> &s1, const std::vector<cv::Point> &s2)
{

    return cv::contourArea(s1) < cv::contourArea(s2);
}

//检测视野中第二大的区域（并检测第二大区域中心是否是蓝色）
bool DetectionByColor::detectBackgroundObject(cv::Mat &sceneImg, cv::RotatedRect &roi_1, cv::RotatedRect &roi_2, 
                                              cv::Scalar hsv_background_l, cv::Scalar hsv_background_h)
{    
     // 将RGB转化为HSV
	cv::Mat hsvImg;
	cv::cvtColor(sceneImg, hsvImg, CV_BGR2HSV);

	// HSV 阈值分割 
	cv::Mat bw;
	inRange(hsvImg, hsv_background_l, hsv_background_h, bw);

	// 进行腐蚀消除一部分噪点 
	cv::Mat erodeImg;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::erode(bw, erodeImg, element);

	// 进行膨胀变回原来的形状
	cv::Mat dilateImg;
	cv::erode(erodeImg, dilateImg, element);

	// 查找轮廓 
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(dilateImg, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());

	// 面积排序 找出最大和第二大面积
	if ( contours.size() > 1 )
	{
		double largest_area_1 = 0;
		double largest_area_2 = 0;
		int largest_contour_index_1 = 0;
		int largest_contour_index_2 = 0;

		// 计算前两个数值，找出临时最大和第二大数值
		if (cv::contourArea(contours[0]) < cv::contourArea(contours[1]))
		{
			largest_area_1 = cv::contourArea(contours[1]);
			largest_area_2 = cv::contourArea(contours[0]);

			largest_contour_index_1 = 1;
			largest_contour_index_2 = 0;
		}
		else
		{
			largest_area_1 = cv::contourArea(contours[0]);
			largest_area_2 = cv::contourArea(contours[1]);

			largest_contour_index_1 = 0;
			largest_contour_index_2 = 1;
		}

		// 计算遍历后面所有元素，找出最终的最大值和第二大数值
		for (size_t i = 2; i < contours.size(); i++)  // 遍历每个轮廓
		{
			if (cv::contourArea(contours[i]) > largest_area_1)
			{
				largest_area_2 = largest_area_1;
				largest_area_1 = cv::contourArea(contours[i]);

				largest_contour_index_2 = largest_contour_index_1;
				largest_contour_index_1 = i;
			}
			else if (cv::contourArea(contours[i]) > largest_area_2)
			{
				largest_area_2 = cv::contourArea(contours[i]);
				largest_contour_index_2 = i;
			}
		}

		// 如果最大区域面积小于4000，证明视野范围之内没有板子
		if (largest_area_1 < 4000)
		{
			//std::cout << "return false_ roi1 面积小于4000" << std::endl;
			return false;
		}

		// 如果最大区域面积大于4000，证明视野进入绿色板子以内
		else
		{
			//最大面积的最小外接矩形
			roi_1 = cv::minAreaRect(contours[largest_contour_index_1]);

			//第二大面积的最小外接矩形
			roi_2 = cv::minAreaRect(contours[largest_contour_index_2]);

			// -------------验证第二大面积包含在第一大面积之内--------------
			
			//调用isInReigon函数判断第二大面积是否包含在最大面积之内
			bool isInReigon = IsInReigon(roi_1, roi_2);
		    
			if (isInReigon)
			{
			
					// 求中心像素的mask平均值
					cv::Point2f center = roi_2.center;

					cv::Scalar hsv_avg = (0, 0, 0);

					for (int i = 0; i < 3; i++)
					{
						hsv_avg[i] = ((hsvImg.at<Vec3b>(center.y - 1, center.x - 1)[i] + hsvImg.at<Vec3b>(center.y, center.x - 1)[i] + hsvImg.at<Vec3b>(center.y + 1, center.x - 1)[i]
							+ hsvImg.at<Vec3b>(center.y - 1, center.x)[i] + hsvImg.at<Vec3b>(center.y, center.x)[i] + hsvImg.at<Vec3b>(center.y + 1, center.x)[i]
							+ hsvImg.at<Vec3b>(center.y - 1, center.x + 1)[i] + hsvImg.at<Vec3b>(center.y, center.x + 1)[i] + hsvImg.at<Vec3b>(center.y + 1, center.x + 1)[i]) / 9);
					}

					if (hsv_avg[0] > 95 && hsv_avg[0] < 130)
					{
						//std::cout << "return true" << std::endl;
						return true;
					}
					else
					{
						//std::cout << "return false_roi_2中心不是蓝色" << std::endl;
						return false;
					}
					
			}
			else
			{
					// std::cout << "return false_ roi2 轮廓超出范围" << std::endl;
					return false;
			}		
		}
			
		
	}
	else if ( contours.size() == 1)
	{
		roi_1 = cv::minAreaRect(contours[0]);
		//std::cout << "return false_contours == 1" << std::endl;
                return false;
	}
	else
	{
		// std::cout << "return false_contours == 0" << std::endl;
		return false;
	}

}

bool DetectionByColor::detectBlackCircle(cv::Mat &input, cv::Point2f &center)
{
        // 缩小图像 
	// resize(input, input, cv::Size(384, 288));
	// resize(src, src, Size(512,384));
	// resize(src, src, Size(448, 336));

	//裁剪图像
	// cv::Rect rect(42, 19, 250, 250);
	// input = input(rect);
	
	// 声明处理过程mat
	cv::Mat midImage;

	// 声明中心点的X、Y
	float sum_x = 0;
	float sum_y = 0;

	 //将图片进行裁剪 
	cvtColor(input, midImage, CV_BGR2GRAY);

	//直方图均衡化
	equalizeHist(midImage, midImage);

	// 阈值分割 减少计算量
	threshold(midImage, midImage, 200, 255, cv::THRESH_TRUNC);

	//中值滤波
	medianBlur(midImage, midImage, 3);

	//高斯滤波
	GaussianBlur(midImage, midImage, cv::Size(3, 3), 2, 2);

	//腐蚀（让圆形轮廓更明显）
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::erode(midImage, midImage, element);
	
	//进行霍夫圆变换
	std::vector<cv::Vec3f> circles;
	HoughCircles(midImage, circles, CV_HOUGH_GRADIENT, 1, 10, 100, 88, 0, 0); //检测宋老师的图片 
        // std::cout << "the number of detected circles = " << circles.size() << std::endl;
	if (circles.size() >= 1)
	{
		for (size_t i = 0; i < circles.size(); i++)
		{         
                        //将每个检测出来的圆心分别叠加起来
			sum_x += circles[i][0];
			sum_y += circles[i][1];
		}
		
		//计算平均圆心
		center.x = sum_x / circles.size();
		center.y = sum_y / circles.size();

		return true;
	}
	else
	{
		return false;
	}
}
bool DetectionByColor::detectPureObject(cv::Mat &sceneImg, cv::RotatedRect &roi,
                                        cv::Scalar hsv_object_l1, cv::Scalar hsv_object_h1,
                                        cv::Scalar hsv_object_l2, cv::Scalar hsv_object_h2)
{
        // 将RGB转化为HSV
	cv::Mat hsvImg;
	cv::cvtColor(sceneImg, hsvImg, CV_BGR2HSV);

	// HSV 阈值分割 (红色分成两个颜色） 
	cv::Mat mask1;
	inRange(hsvImg, hsv_object_l1, hsv_object_h1, mask1);

	cv::Mat mask2;
	inRange(hsvImg, hsv_object_l2, hsv_object_h2, mask2);

	cv::Mat bw = mask1 | mask2;
  
	// 进行腐蚀消除一部分噪点 
	cv::Mat erodeImg;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::erode(bw, erodeImg, element);

	// 进行膨胀变回原来的形状
	cv::Mat dilateImg;
	cv::dilate(erodeImg, dilateImg, element);

	// 查找轮廓 
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(dilateImg, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());

	// 如果查找到至少一个轮廓则进行判定 
	if (contours.size() >= 1)
	{
		// 查找轮廓当中最大的填充面积
		int largest_area_1 = 0;
		int largest_contour_index_1 = 0;

		for (size_t i = 0; i< contours.size(); i++)  // 遍历每个轮廓
		{
			double area_1 = cv::contourArea(contours[i]);  // 计算每个轮廓的面积

			if (area_1 > largest_area_1)
			{
				largest_area_1 = area_1;
				largest_contour_index_1 = i;               // 储存最大轮廓的数字
			}
			
		}
                // std::cout << "largest_area_1 = " << largest_area_1 << std::endl;
		// 如果最大区域面积小于4000，证明视野范围之内没有小人
		if (largest_area_1 < 500)
		{
			return false;
		}
		else
		{
			//最大面积的最小外接矩形
			roi = cv::minAreaRect(contours[largest_contour_index_1]);
			
			return true;
		}
	}
	else
	{
		return false;
	}
}
