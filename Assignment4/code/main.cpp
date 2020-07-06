#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}


cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
	int tnum = control_points.size();
	int combination_num = 1;
	cv::Point2f ans;
	float tpow = 1;
	float ftpow = pow(1 - t, tnum - 1);
	for (int i = 0; i < tnum; ++i)
	{
		if (i != 0)
		{
			combination_num = combination_num * (tnum - i) / i;
			tpow *= t;
			ftpow /= (1 - t);
			
		}
	//	std::cout << combination_num << "  " << tpow << "  " << ftpow << std::endl;
		ans += control_points[i] * combination_num * tpow * ftpow;
	}
    return ans;
}

float dist2Point(cv::Point2f& lhs, cv::Point2f& rhs)
{
	return sqrt(pow(lhs.x - rhs.x, 2) + pow(lhs.y - rhs.y, 2));
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
	std::vector<std::vector<float>> weights(window.rows, std::vector<float>(window.cols, 0));
	cv::Point2f bpoint = control_points[0];
	cv::Point2f epoint = control_points[control_points.size() - 1];
	window.at<cv::Vec3b>(bpoint.y, bpoint.x)[0] = 255;
	window.at<cv::Vec3b>(bpoint.y, bpoint.x)[1] = 255;
	window.at<cv::Vec3b>(epoint.y, epoint.x)[2] = 255;
	auto bcolor = window.at<cv::Vec3b>(bpoint.y, bpoint.x);
	auto ecolor = window.at<cv::Vec3b>(epoint.y, epoint.x);
	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		auto point = recursive_bezier(control_points, t);
	//	std::cout << point.x << "   " << point.y << std::endl;
	//	window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
		float dist1 = dist2Point(bpoint, point);
		float dist2 = dist2Point(epoint, point);
		auto tcolor = (dist1 / (dist1 + dist2) * ecolor + dist2 / (dist1 + dist2) * bcolor);
		int px = point.x, py = point.y;
		for(int dx=0;dx<2;++dx)
			for (int dy = 0; dy < 2; ++dy)
			{
				int neib_px = px + dx, neib_py = py + dy;
				cv::Point2f neib_point(neib_px, neib_py);
				float disneib = dist2Point(point, neib_point);
				float rate = weights[neib_py][neib_px] / (weights[neib_py][neib_px] + disneib);
				weights[neib_py][neib_px] += disneib;
				window.at<cv::Vec3b>(neib_py, neib_px) = window.at<cv::Vec3b>(neib_py, neib_px)*rate + disneib / weights[neib_py][neib_px] * tcolor;
			}
	}

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
           // naive_bezier(control_points, window);
			bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve1.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}