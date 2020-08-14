#include "Material.hpp"


//wi,wo请注意，这个是路径追踪，从摄像机到光源
//但是资料中一般都是光源到摄像机计算，如必要，请交换

float Material::Fresnels(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector3f &h)
{
	//其实如果算上kr/ks(反射/透射)的话，还要乘一个比例
	//这里是ks/kd（高光/散射）,F0好像是垂直的时候？
	switch (m_type)
	{
	case BSDF:
	case Cook_Torrance:
	{
		float ansF = 1.0f;
		if (dotProduct(h, wo) > 0)
			ansF = F0 + (1.0f - F0)*pow5(1 - abs(dotProduct(h, wo)));//这个不应该加这个的，为了调试bsdf
		else
			ansF = F0 + (1.0f - F0)*pow5(1 - abs(dotProduct(h, wi)));
		return std::max(0.0f, std::min(1.0f, ansF));
		break;
	}
	case GGX:
		return F0 + (1.0f - F0)*pow5(1 - dotProduct(h, wo));
		break;
	/*case BSDF:
	{
		float t_ni = this->ni, t_nt = this->nt;
		if (dotProduct(wo, N) < 0)
			std::swap(t_ni, t_nt);
		float c = abs(dotProduct(wo,h));
		float g = pow2(t_nt / t_ni) - 1 + c * c;
		//全反射现象
		if (g < 0)
			return 1.f;
		g = sqrtf(g);
		float F = pow2(g-c) / (2 * pow2(g+c));
		F = F * (1 + pow2(c*(g + c) - 1) / pow2(c*(g - c) - 1) );
		//std::cout << "this is Fresnels : " << F << std::endl;
		return F;
		break;
	}*/
	}
	return 1.0f;
}

float Material::NormalDistrFunc(const Vector3f &wi, const Vector3f&wo, const Vector3f &N, const Vector3f &h)
{
	//N是宏观法线，h是半程向量，m是表面粗糙程度
	//是Beckmann分布
	float cosSita = dotProduct(N, h);
	float cosSita_p2 = cosSita * cosSita;
	//std::cout << "this is cos : " << cosSita << std::endl;
	
	float m_p1 = this->roughness ,m_p2 = this->roughness * this->roughness;
	switch (m_type)
	{
	//case BSDF:
	case Cook_Torrance:
	{
		return exp(((cosSita_p2 - 1) / cosSita_p2) / m_p2) /
			(M_PI * m_p2 * cosSita_p2 * cosSita_p2);
		break;
	}
	case BSDF:
	case GGX:
	{
		float N_d = characFc(cosSita) / M_PI *
			pow2(m_p1 / (cosSita_p2*(m_p2 - 1) + 1));
	//	std::cout << "this is N : " << N_d << std::endl;
		return N_d;
		break;
	}
	}
	return 0.0;
}

float Material::GeomAtteFactor(const Vector3f &wi, const Vector3f& wo, const Vector3f &N, const Vector3f &h)
{
	//是Beckmann分布，这个是光线遮蔽
	//这个对于背面为0
	//if (dotProduct(wi, h)*dotProduct(wi, N) < 0 || dotProduct(wo, h)*dotProduct(wo, N) < 0)
	//	return 0;
	switch (m_type)
	{
	case Cook_Torrance:
	{

		float Gb = 2 * dotProduct(N, h)*dotProduct(N, wi) / dotProduct(wi, h);
		float Gc = 2 * dotProduct(N, h)*dotProduct(N, wo) / dotProduct(wo, h);
		return std::min(1.0f, std::min(Gb, Gc));
		break;
	}
	case GGX:
	case BSDF:
	{
		float cos_i = dotProduct(wi, N), cos_o = dotProduct(wo, N), ag = this->roughness;
		float G1 = 2 * characFc(dotProduct(wi, h) / cos_i) / (1 + sqrtf(1 + ag * ag*(1 - cos_i * cos_i)/(cos_i*cos_i)));
		float G2 = 2 * characFc(dotProduct(wo, h) / cos_o) / (1 + sqrtf(1 + ag * ag*(1 - cos_o * cos_o)/(cos_o*cos_o)));
		//std::cout << "this is G : " << G1 << "   " << G2 << std::endl;
		return G1 * G2;
	}
	}
	return 0.0;
}

Vector3f Material::sample(const Vector3f &wi, const Vector3f &N,float& m_pdf) {
	switch (m_type) {
	case BSDF:
	{
		float m_p2 = this->roughness * this->roughness;
		float x_1 = get_random_float(), x_2 = get_random_float(),x_3 = get_random_float();
		float z = sqrtf( (1 - x_1) / (x_1*(m_p2 - 1) + 1) );
		float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
		Vector3f h(r*std::cos(phi), r*std::sin(phi), z);
		h = normalize(toWorld(h, N));
		float cosSita = dotProduct(h, N);
		float F = Fresnels(wi, wi, N, h);
		m_pdf = 2.0f*m_p2*z / (2 * M_PI*pow2(1 + z * z*(m_p2 - 1)));
	//	std::cout << "this is Fresnels : " << F << std::endl;
		bool isreflect = true;

		if (x_3 > F)
			isreflect = false;
		//std::cout << "this cos h and N : " << dotProduct(h, N) << std::endl;
		Vector3f wo(0);
		wo = normalize(refract(-wi, h, this->m_nt));
		if (wo.norm() < EPSILON)
			isreflect = true;
		if (isreflect)
		{
			wo = normalize(reflect(wi, h));
			if (dotProduct(wi, N) * dotProduct(wo, N) < 0)
			{
				m_pdf = -1.0;
			//	std::cout << "this is reflect : " << dotProduct(wi, N)<<"  "<<dotProduct(wo,N)<< std::endl;
			}
			else
			{
				m_pdf *= 1;//F / (4.f*abs(dotProduct(wi, h)));
			//	std::cout << "this is reflect : " << dotProduct(wi, N)<<"  "<<dotProduct(wo,N)<< std::endl;
			}
		}
		else
		{
			float ni = this->m_ni, nt = this->m_nt;
			wo = normalize(refract(-wi, h, nt));
		//	std::cout << "this is h/N  " << dotProduct(h, N) << std::endl;
		//	std::cout << "this is F  " <<  F << std::endl;
			if (dotProduct(wo, N) < 0)
				std::swap(ni, nt);
			if (dotProduct(wi, N) * dotProduct(wo, N) > 0 || dotProduct(wi,N)*dotProduct(wi,h)<0 || dotProduct(wo, N)*dotProduct(wo, h) < 0)
			{
				m_pdf = -1.0;
				//std::cout << "this refract : " << dotProduct(wi, h) << "   " << dotProduct(wo, h) << std::endl;
			}
			else
			{
				m_pdf *= 1;//(1 - F)*nt*nt*abs(dotProduct(wi, h)) / pow2(ni*dotProduct(wo, h) + nt * dotProduct(wi, h));
			//	std::cout << "this refract with h : " << dotProduct(wi, h) << "   " << dotProduct(wo, h) << std::endl;
			//	std::cout << "this refract with N : " << dotProduct(wi, N) << "   " << dotProduct(wo, N) << std::endl;
		//		std::cout << "this wi,wo : " << dotProduct(wi, wi) << "   " << dotProduct(wo, wo) << std::endl;
				auto hpp = -normalize(wo*ni + wi * nt);
				auto hppf = -normalize(wo*nt + wi * ni);
				//std::cout << "this is htCos  fan:  " << dotProduct(hpp, h) <<"   "<<dotProduct(hppf,h) << std::endl;
				if (abs(dotProduct(hpp, h) - 1) > EPSILON)
					std::cout << "this is fxck" << std::endl;
			//	std::cout << "this is pdf : " << m_pdf << std::endl;
			
			}
		}
	//	std::cout << "this is pdf : " << m_pdf << std::endl;
		return wo;
		break;
	}
	case DIFFUSE:
	{
		// uniform sample on the hemisphere
		float x_1 = get_random_float(), x_2 = get_random_float();
		float z = std::fabs(1.0f - 2.0f * x_1);
		float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
		Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
		Vector3f wo = toWorld(localRay, N);
		m_pdf = pdf(wi, wo, N);
		return wo;
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
		Vector3f wo = toWorld(localRay, N);
		m_pdf = pdf(wi, wo, N);
		return wo;
		break;
	}
	case Cook_Torrance:
	{
		float x_1 = get_random_float(), x_2 = get_random_float();
		float z = std::fabs(1.0f - 2.0f * x_1);
		float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
		Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
		Vector3f wo = toWorld(localRay, N);
		m_pdf = pdf(wi, wo, N);
		return wo;
		break;
	}
	}
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) {
	switch (m_type) {
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
	case Cook_Torrance:
	case DIFFUSE_COS:
	{
		float c = 0.0f;
		if ((c = dotProduct(wo, N)) > EPSILON)
		{
			return c / M_PI;
		}
		else
			return 0.0f;
		break;
	}
	case BSDF:
	{
		return 0.25f / M_PI;
	}

	}
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) {
	switch (m_type) {
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
		if (cosalpha > 0.0f)
		{
			Vector3f hr = normalize(wi + wo);
			Vector3f ks = Fresnels(wi, wo, N, hr);
		//	float n1 = dotProduct(N, wo), n2 = dotProduct(N, wi);
			Vector3f specularW = NormalDistrFunc(wi, wo, N, hr)*ks*GeomAtteFactor(wi, wo, N, hr) /
				(4 * dotProduct(N, wo)*dotProduct(N, wi));
			Vector3f diffuseW = (Vector3f(1) - ks)*(1 - metal)*Kd / M_PI;
			
			//std::cout << "this is specular: " << specularW << std::endl;
			return specularW + diffuseW;
		}
		else
			return Vector3f(0);
		break;
	}
	case BSDF:
	{
		float ni = this->m_ni, no = this->m_nt;
		if (dotProduct(N,wi)*dotProduct(N, wo) > 0)
		{
			Vector3f hr = normalize(wi + wo);
			if (dotProduct(hr, N) < 0)
				hr = -hr;
			Vector3f F = Fresnels(wi, wo, N, hr);
			Vector3f fr = NormalDistrFunc(wi, wo, N, hr)* F* GeomAtteFactor(wi, wo, N, hr) /
				(4 * dotProduct(N, wo)*dotProduct(N, wi));
		//	std::cout << "this is BSDF fr 项----------------->" << fr << std::endl;
			return fr;
		}
		else
		{
			if (dotProduct(N, wo) < 0)
			{
			//	std::cout << "this is wi/N " << dotProduct(wi, N) << std::endl;
			//	std::cout << "this is wo/N " << dotProduct(wo, N) << std::endl;
			//	std::cout << " --------------------- " << std::endl;
				std::swap(ni, no);
			//	std::cout << "this is inside ft " << std::endl;
			}
			Vector3f ht = -normalize(wo * ni + no * wi);
			
		//	std::cout << "this is N/h - eval " << dotProduct(ht, N) << std::endl;
			if (true && dotProduct(ht, N) < 0)
			{
				return Vector3f(0.0);
				std::cout << "this is wi/h " << dotProduct(wi, ht) << std::endl;
				std::cout << "this is wo/h " << dotProduct(wo, ht) << std::endl;
				std::cout << "this is wi/N " << dotProduct(wi, N) << std::endl;
				std::cout << "this is wo/N " << dotProduct(wo, N) << std::endl;
				std::cout << "this is N/h " << dotProduct(N, ht) << std::endl;
				auto hpp = -normalize(wo * no + ni * wi);
				std::cout << "this is hpp " << dotProduct(N, hpp) << std::endl;
				std::cout << " ======================= " << std::endl;
			}
				
			Vector3f ft = abs(dotProduct(wi, ht)) * abs(dotProduct(wo, ht)) *
				no*no*(Vector3f(1.0f) - Fresnels(wi, wo, N, ht))*GeomAtteFactor(wi, wo, N, ht)*NormalDistrFunc(wi, wo, N, ht) /
				(abs(dotProduct(wi, N))*abs(dotProduct(wo, N))*pow2(no*dotProduct(wi, ht) + ni * dotProduct(wo, ht)));
			if (false )
			{
				std::cout << "this is BSDF ft 项----------------->" << ft << std::endl;
				std::cout << "this is F : " << Fresnels(wi, wo, N, ht) << std::endl;
				std::cout << "this is G : " << GeomAtteFactor(wi, wo, N, ht) << std::endl;
				std::cout << "this is D : " << NormalDistrFunc(wi, wo, N, ht) << std::endl;
			}
			return ft;
		}
		break;
		
	}
	}
	return Vector3f(0);
}