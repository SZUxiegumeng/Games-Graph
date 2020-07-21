//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
//	std::cout << "this is p : " << p << std::endl;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
		//	std::cout << "now area is  : " << emit_area_sum << std::endl;
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
		//		std::cout << "this final pdf is : " << pdf << std::endl;
		//		std::cout << "============================" << std::endl;
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here
	//这个是物体的碰撞点
	Intersection pInter = Scene::intersect(ray);
	Vector3f L_dir;
	if(!pInter.happened || dotProduct(pInter.normal,ray.direction)>EPSILON)
		return Vector3f();
	else if (depth==0 && pInter.obj->hasEmit())
	{
		auto L = pInter.m->getEmission() * -dotProduct(ray.direction, pInter.normal);
		return L;
	}
	else
	{
		Vector3f pToEyeDir = normalize(ray.direction);
		Intersection LightInter;
		float Light_pdf_p1,Sample_pdf_p1 = 0.0;
		sampleLight(LightInter, Light_pdf_p1);
		Vector3f pToLightDir = normalize(LightInter.coords - pInter.coords);
		//float pToLightT = (LightInter.coords - pInter.coords).norm();
		Ray pToLightRay(pInter.coords + pToLightDir * EPSILON, pToLightDir);

		//这个是MIS生成的BRDF采样
		//Vector3f pSampleDir = normalize(pInter.m->sample(ray.direction, pInter.normal));
		//Ray pSampleRay(pInter.coords + EPSILON * pSampleDir, pSampleDir);
		//Intersection sampleIntersect = Scene::intersect(pSampleRay);
		//这个是判断是不是背面
		if ( dotProduct(pToLightRay.direction, pInter.normal) > EPSILON && dotProduct(pToLightRay.direction, LightInter.normal) < EPSILON)
		{
			Intersection pTolightTravel = Scene::intersect(pToLightRay);
			//没碰到，或者没碰在光源和P之间
			if (!pTolightTravel.happened || (pTolightTravel.coords - LightInter.coords).norm() <= EPSILON)
			{
			//	if (sampleIntersect.happened && sampleIntersect.obj->hasEmit())
			//		Sample_pdf_p1 = Light_pdf_p1;
			//	else
			//		Sample_pdf_p1 = 0.0;
				//计算直接光照
				
				L_dir = LightInter.emit * pInter.m->eval(ray.direction, pToLightRay.direction, pInter.normal)
					*dotProduct(pToLightRay.direction, pInter.normal)
					*dotProduct(-pToLightRay.direction, LightInter.normal)
					/ pow((LightInter.coords - pInter.coords).norm(), 2)
					/ (Light_pdf_p1  ); //这个是
				
				if (false && L_dir.norm() > 1)
				{
					std::cout << "this is boll L_dir  " << L_dir << std::endl;
					std::cout << "=============start=================" << std::endl;
					std::cout << pInter.m->eval(ray.direction, pToLightRay.direction, pInter.normal) << std::endl;
					std::cout << dotProduct(pToLightRay.direction, pInter.normal) << std::endl;
					std::cout << dotProduct(-pToLightRay.direction, LightInter.normal) << std::endl;
					std::cout << pow((LightInter.coords - pInter.coords).norm(), 2) << std::endl;
					std::cout << Light_pdf_p1 << std::endl;
					std::cout << "=============end=================" << std::endl;
				}
				
				//+ (pInter.m->pdf(ray.direction, pToLightRay.direction, pInter.normal)) 这个加起来有问题？
				//现在看起来是这个值太大了
			}
		}
		// MIS的BRDF光线计算
		/*if (sampleIntersect.happened && sampleIntersect.obj->hasEmit())
		{
			L_dir += castRay(pSampleRay, depth + 1) * pInter.m->eval(ray.direction, pSampleRay.direction, pInter.normal)
				* dotProduct(pSampleRay.direction, pInter.normal)
				/ (pInter.m->pdf(ray.direction, pSampleRay.direction, pInter.normal) + Sample_pdf_p1);
		}*/
		
		
	}
	Vector3f L_indir;
	//这里俄罗斯轮盘赌
	if (get_random_float() > this->RussianRoulette)
	{
		return L_dir + L_indir;
	}
	Vector3f pSampleinDir = normalize(pInter.m->sample(ray.direction, pInter.normal));
	//std::cout << "this is indirect light : " << pSampleDir.x << "  " << pSampleDir.y << "  " << pSampleDir.z << std::endl;
	Ray pSampleinRay(pInter.coords + EPSILON* pSampleinDir, pSampleinDir);
	Intersection insampleIntersect = Scene::intersect(pSampleinRay);
	//间接光照
	if (true && dotProduct(pInter.normal,pSampleinRay.direction)>EPSILON && insampleIntersect.happened && !insampleIntersect.obj->hasEmit())
	{
		L_indir = castRay(pSampleinRay, depth + 1) * pInter.m->eval(ray.direction, pSampleinRay.direction, pInter.normal)
			* dotProduct(pSampleinRay.direction, pInter.normal)
			/ pInter.m->pdf(ray.direction, pSampleinRay.direction, pInter.normal)
			/ this->RussianRoulette;
	}
	//auto L = L_dir + L_indir;
	//if(L.x>1 || L.y>1 ||L.z>1)
	//	std::cout << "this is all Light " <<L << std::endl;
	return L_dir + L_indir;
}