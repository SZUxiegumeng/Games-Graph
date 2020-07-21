//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, DIFFUSE_COS ,Cook_Torrance};

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

	float D_GGX(const Vector3f &N, const Vector3f &wo, float alpha)
	{
		//这个微表面法线分布函数GGX
		auto x_tran = [](float x) {return x > 0 ? 1.0 : 0.0; };
		float cosSita = dotProduct(N, wo);
		float cosSita_p2 = cosSita * cosSita;
		float alpha_p2 = alpha * alpha;

		return alpha_p2 * x_tran(cosSita) /
			( M_PI * cosSita_p2 * cosSita_p2 *
			(alpha_p2 * ((1 - cosSita_p2) / cosSita_p2)) );
	}
	float D_Beckmann(const Vector3f &N, const Vector3f &h, float m)
	{
		//N是宏观法线，h是半程向量，m是表面粗糙程度
		//是Beckmann分布
		float cosSita = dotProduct(N, h);
		float cosSita_p2 = cosSita * cosSita;
		float m_p2 = m * m;

		return exp((( cosSita_p2-1) / cosSita_p2) / m_p2) /
			(M_PI * m_p2 * cosSita_p2 * cosSita_p2);
	}
	float G_Beckmann(const Vector3f &N, const Vector3f &wi, const Vector3f& wo)
	{
		//是Beckmann分布，这个是光线遮蔽
		Vector3f we = -wi;
		Vector3f h = normalize(we + wo);

		float Gb = 2 * dotProduct(N, h)*dotProduct(N, we) / dotProduct(we, h);
		float Gc = 2 * dotProduct(N, h)*dotProduct(N, wo) / dotProduct(wo, h);

		return std::min(1.0f, std::min(Gb, Gc));
	}

	Vector3f F_Schlick(const Vector3f &F0,const Vector3f &h,const Vector3f &wi )
	{
		//其实如果算上kr/ks(反射/透射)的话，还要乘一个比例
		//这里是ks/kd（高光/散射）,F0好像是垂直的时候？
		Vector3f we = -wi;
		return F0 + (Vector3f(1) - F0)*pow(1 - dotProduct(h, we), 5);
	}
    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            break;
        }
		case DIFFUSE_COS:
		{
			float x_1 = get_random_float(), x_2 = get_random_float();
			float z = (1.0f - 2.0f * x_1);
			z = cos(acos(z) / 2);
			float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
			//	std::cout << "this is r and phi : " << r << "  " << phi << std::endl;
			Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
			//	std::cout << "localRay : " << localRay.x << "  " << localRay.y << "  " << localRay.z << std::endl;
		//	std::cout << "this is N_wo : " << dotProduct(localRay, Vector3f(0, 0, 1)) << std::endl;
			return toWorld(localRay, N);
			break;
		}
		case Cook_Torrance:
		{
			float x_1 = get_random_float(), x_2 = get_random_float();
			float z = std::fabs(1.0f - 2.0f * x_1);
			float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
			Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
			return toWorld(localRay, N);
			break;
		}
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
			//这个是正比cos，但实际上还是要有f
			
        }
		case DIFFUSE_COS:
		{
			float c = EPSILON;
			if ((c = dotProduct(wo, N)) > EPSILON)
			{
				return c / M_PI;
			}
			else
				return 0.0f;
			break;
		}
		case Cook_Torrance:
		{
			if (dotProduct(wo, N) > 0.0f)
				return 0.5f / M_PI;
			else
				return 0.0f;
			break;
		}
			
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
		//	std::cout << "this is cosalpha  " << cosalpha << std::endl;
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
		case DIFFUSE_COS:
		{
			//这个是散射的分布，照抄上面
			float cosalpha = dotProduct(N, wo);
			if (cosalpha > 0.0f) {
				Vector3f diffuse = Kd / M_PI;
				return diffuse;
			}
			else
				return Vector3f(0.0f);
			break;
		}
		case Cook_Torrance:
		{
			float cosalpha = dotProduct(N, wo);
			if (cosalpha>0.0f)
			{
				Vector3f h = normalize(-wi + wo);
				Vector3f F0(0.4);
				Vector3f ks = F_Schlick(F0, h, wi);
				float m = 0.5, metal = 0;
				float D1 = D_Beckmann(N, h, m), G1 = G_Beckmann(N, wi, wo);
				float n1 = dotProduct(N, wo), n2 = dotProduct(N, -wi);
				Vector3f specularW = D_Beckmann(N, h, m)*ks*G_Beckmann(N, wi, wo) /
					(4 * dotProduct(N, wo)*dotProduct(N, -wi));
				Vector3f diffuseW = (Vector3f(1) - ks)*(1 - metal)*Kd / M_PI;
				/*if ((specularW + diffuseW).norm() > 1)
				{
					std::cout << "this is F  " << ks << std::endl;
					std::cout << "this is specularW  " << specularW << std::endl;
					std::cout << "this is diffuseW  " << diffuseW << std::endl;
					std::cout << "D1  " << D1 << "  G1  " << G1 << "  n1  " << n1 << "  n2  " << n2 << std::endl;
					std::cout << "============== end =============" << std::endl;
				}*/
			
				return specularW + diffuseW;
			}
			else
				return Vector3f(0);
			break;
		}
    }
}


#endif //RAYTRACING_MATERIAL_H
