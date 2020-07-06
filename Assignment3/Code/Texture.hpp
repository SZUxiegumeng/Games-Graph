//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;
	Eigen::Vector3f getColorBilinear(float u, float v)
	{
		auto u_img = u * width;
		auto v_img = (1 - v) * height;
		cv::Vec3b color;
		if (u < 1 && v < 1)
		{
			cv::Vec3b& lb = image_data.at<cv::Vec3b>(v_img, u_img);
			cv::Vec3b& rb = image_data.at<cv::Vec3b>(v_img, u_img+1);
			cv::Vec3b& lt = image_data.at<cv::Vec3b>(v_img+1, u_img);
			cv::Vec3b& rt = image_data.at<cv::Vec3b>(v_img+1, u_img+1);
			cv::Vec3b midTop = lt + (rt - lt)*(u_img - floor(u_img));
			cv::Vec3b midBot = lb + (rb - lb)*(u_img - floor(u_img));
			color = midBot + (midTop - midBot) * (v_img - floor(v_img));
		}else
			color = image_data.at<cv::Vec3b>(v_img, u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}
    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
