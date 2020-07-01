// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

inline float Cross2D(float x1, float y1, int x2, float y2)
{
	return x1 *y2 - y1*x2;
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	//这里选择用叉乘正负号相同来判别
	float cross1 = Cross2D(x - _v[0].x(), y - _v[0].y(), _v[1].x() - _v[0].x(), _v[1].y() - _v[0].y());
	float cross2 = Cross2D(x - _v[1].x(), y - _v[1].y(), _v[2].x() - _v[1].x(), _v[2].y() - _v[1].y());
	float cross3 = Cross2D(x - _v[2].x(), y - _v[2].y(), _v[0].x() - _v[2].x(), _v[0].y() - _v[2].y());
	return (cross1 > 0 && cross1 > 0 && cross3 > 0) || (cross1 < 0 && cross2 < 0 && cross3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
	float superMatrixLength = 1.0 / (2 * superRate);

	float LeftBound = std::min(v[0].x(), std::min(v[1].x(),v[2].x()));
	float RightBound = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
	float BottomBound = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
	float TopBound = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));
	
	for(int x=floor(LeftBound);x<= ceil(RightBound);++x )
		for (int y = floor(BottomBound); y <= ceil(TopBound); ++y)
		{
			float sx = (x - 0.5) + superMatrixLength, sy = (y - 0.5) + superMatrixLength;
			int BufferDirty = 0;
			Vector3f super_color = Vector3f(0, 0, 0);
	//		std::cout << "==============================" << std::endl;
			for (int di = 0; di < superRate*superRate; ++di)
			{
				float dx =  2.0 * (di%superRate)*superMatrixLength + sx;
				float dy =  2.0 * (di/superRate)*superMatrixLength + sy;
			//	std::cout << "dx : " << dx << "   dy  :  " << dy << std::endl;
				//auto[alpha,beta,gamma] = computeBarycentric2D(x, y, t.v);
				//这段是计算深度的
				auto paramBarycentric = computeBarycentric2D(dx, dy, t.v);
				float alpha = std::get<0>(paramBarycentric), beta = std::get<1>(paramBarycentric), gamma = std::get<2>(paramBarycentric);
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;

				int tpos = get_index(x, y, superRate) + di;
				if (insideTriangle(dx, dy, t.v) && z_interpolated < super_depth_buf[tpos])
				{
					super_frame_buf[tpos] = t.getColor();
					super_depth_buf[tpos] = z_interpolated;
					BufferDirty += 1;
				}
				super_color += super_frame_buf[tpos];
			//	if ( BufferDirty != superRate*superRate)
			//		super_color << 0, 0, 0;
			}
			if (BufferDirty)
			{
				set_pixel(Eigen::Vector3f(x, y, 1.0), super_color/(superRate*superRate));
			}
			
			
		}
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
   

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0});
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		//这里加上超采样的初始化代码
		std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h,int sRate) : width(w), height(h), superRate(sRate)
{
	superRate = 10;
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
	//这里可以自定义超采样的倍数
	super_depth_buf.resize(superRate*superRate*w*h);
	super_frame_buf.resize(superRate*superRate*w*h);
}

//这里我添加了参数，是为了适应超采样
int rst::rasterizer::get_index(int x, int y,int sRate)
{
    return ((height-1-y)*width + x)*sRate;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

void rst::rasterizer::set_superRate(int sRate)
{
	superRate = sRate;
}

// clang-format on