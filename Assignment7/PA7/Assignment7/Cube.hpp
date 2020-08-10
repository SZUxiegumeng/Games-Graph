//
// Created by LEI XU on 5/13/19.
//

#ifndef RAYTRACING_CUBE_H
#define RAYTRACING_CUBE_H

#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"

class Cube : public Object {
public:
	Vector3f pMin,pMax;
	Material *m;
	float area;
	Cube(const Vector3f &_pmin,const Vector3f& _pmax, Material* mt = new Material()):pMin(_pmin),pMax(_pmax),m(mt)
	{
		float e1 = pMax[0] - pMin[0], e2 = pMax[1] - pMin[1], e3 = pMax[2] - pMin[2];
		area = 2 * (e1*e2 + e1 * e3 + e2 * e3);
	}
	bool intersect(const Ray& ray) {
		// analytic solution
		float tnear;
		uint32_t index;
		return intersect(ray, tnear, index);
	}
	bool intersect(const Ray& ray, float &tnear, uint32_t &index) const
	{
		// analytic solution
		std::array<int, 3> dirIsNeg{ int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0) };
		float t[6];
		for (int i = 0; i < 3; ++i)
		{
			t[2 * i + !dirIsNeg[i]] = (pMin[i] - ray.origin[i]) * ray.direction_inv[i];
			t[2 * i + dirIsNeg[i]] = (pMax[i] - ray.origin[i]) * ray.direction_inv[i];
		}
		float tenter = std::max(t[0], std::max(t[2], t[4]));
		float texit = std::min(t[1], std::min(t[3], t[5]));
		//std::cout << "tenter  and  texit " << tenter << "   " << texit << std::endl;
		if (texit >= 0 && tenter <= texit)
		{
			if (tenter < 0)
				tenter = texit;
			tnear = tenter;
			return true;
		}
		return false;
	}
	Intersection getIntersection(Ray ray) {
		Intersection result;
		result.happened = false;
		float tnear;
		uint32_t index;
		if (!intersect(ray, tnear, index))
			return result;
		result.happened = true;

		result.coords = Vector3f(ray.origin + ray.direction * tnear);
		Vector3f tn = Vector3f();
		for (int i = 0; i < 3; ++i)
		{
			tn[i] = int(result.coords[i] > pMin[i] + EPSILON) + int(result.coords[i] > pMax[i] - EPSILON) - 1;
		}
		result.normal = normalize(tn);
	//	if (result.normal.norm() < EPSILON)
	//		std::cout << "this is BUG" << std::endl;
		result.m = this->m;
		result.obj = this;
		result.distance = tnear;
		return result;

	}
	void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const
	{}

	Vector3f evalDiffuseColor(const Vector2f &st)const {
		//return m->getColor();
		return Vector3f(0.5);
	}
	Bounds3 getBounds() {
		return Bounds3(pMin,pMax);
	}
	void Sample(Intersection &pos, float &pdf) {
	}
	float getArea() {
		return area;
	}
	bool hasEmit() {
		return m->hasEmission();
	}
};




#endif //RAYTRACING_CUBE_H
