//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H

typedef unsigned char u08;
#define MY_PI 3.1415926
#define TWO_PI (2.0* MY_PI)

inline float max(float lhs, float rhs)
{
	return lhs > rhs ? lhs : rhs;
}
inline float min(float lhs, float rhs)
{
	return lhs > rhs ? rhs : lhs;
}
#endif //RASTERIZER_GLOBAL_H
