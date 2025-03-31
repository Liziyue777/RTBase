#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		float etaI = iorInt;
		float etaT = iorExt;

		if (cosTheta < 0.0f)
		{
			std::swap(etaI, etaT);
			cosTheta = -cosTheta;
		}

		float sinThetaT = etaI / etaT * sqrtf(std::max(0.0f, 1.0f - cosTheta * cosTheta));
		if (sinThetaT >= 1.0f)
		{
			// full
			return 1.0f;
		}
		float cosThetaT = sqrtf(std::max(0.0f, 1.0f - sinThetaT * sinThetaT));
		float rs = (etaI * cosTheta - etaT * cosThetaT) / (etaI * cosTheta + etaT * cosThetaT);
		float rp = (etaT * cosTheta - etaI * cosThetaT) / (etaT * cosTheta + etaI * cosThetaT);
		return 0.5f * (rs * rs + rp * rp);
	}

	static Colour fresnelConductor(float cosTheta, Colour eta, Colour k)
	{
		if (cosTheta < 0.0f) cosTheta = 0.0f;
		if (cosTheta > 1.0f) cosTheta = 1.0f;

		float cos2 = cosTheta * cosTheta;
		float sin2 = 1.0f - cos2;

		// eta2 + k2
		Colour eta2 = eta * eta;
		Colour k2 = k * k;

		// Common terms
		Colour t0 = eta2 - k2;
		Colour t1 = t0 - Colour(sin2, sin2, sin2);
		Colour a2plusb2 = (eta2 + k2);

		Colour Rparl2 = ((t1 * t1) + Colour(4.0f, 4.0f, 4.0f) * eta2 * k2) / (a2plusb2 * a2plusb2);
		Colour Rperp2 = ((a2plusb2 - Colour(2.0f * eta.r * cosTheta, 2.0f * eta.g * cosTheta, 2.0f * eta.b * cosTheta)) *
			(a2plusb2 - Colour(2.0f * eta.r * cosTheta, 2.0f * eta.g * cosTheta, 2.0f * eta.b * cosTheta))) /
			(a2plusb2 * a2plusb2);

		return (Rparl2 + Rperp2) * 0.5f;
	}

	static float lambdaGGX(Vec3 wi, float alpha)
	{
		float absTanTheta = fabsf(tanf(acosf(wi.z)));
		if (absTanTheta == 0.0f) return 0.0f;
		float a = 1.0f / (alpha * absTanTheta);
		if (a < 1.6f)
		{
			return (1.0f - 1.259f * a + 0.396f * a * a) / (3.535f * a + 2.181f * a * a);
		}
		return 0.0f;
	}

	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		return 1.0f / (1.0f + lambdaGGX(wi, alpha) + lambdaGGX(wo, alpha));
	}

	static float Dggx(Vec3 h, float alpha)
	{
		float cosThetaH = h.z;
		float cos2Theta = cosThetaH * cosThetaH;
		float tan2Theta = (1.0f - cos2Theta) / cos2Theta;
		float alpha2 = alpha * alpha;

		float denom = M_PI * cos2Theta * cos2Theta * (alpha2 + tan2Theta) * (alpha2 + tan2Theta);
		return alpha2 / denom;
	}

	static Vec3 reflect(const Vec3& I, const Vec3& N) // GlassBSDF
	{
		return I - N * (2.0f * Dot(I, N));
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};

//(done)
class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here (done)
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here (done)
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};
//(done)
class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code (done)
	
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
		Vec3 wi = shadingData.frame.toWorld(wiLocal);
		pdf = 1.0f;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code (done)
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF (done)
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

//AS:Add Glass BRDF 
class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;

	GlassBSDF() = default;

	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wo = shadingData.wo;
		Vec3 n = shadingData.sNormal;

		bool entering = Dot(wo, n) > 0.0f;
		float etaI;
		float etaT;
		Vec3 normal;

		if (entering)
		{
			etaI = extIOR;
			etaT = intIOR;
			normal = n;
		}
		else
		{
			etaI = intIOR;
			etaT = extIOR;
			normal = -n;
		}

		float eta = etaI / etaT;
		float cosThetaI = Dot(wo, normal);

		// Fresnel
		float fresnel = ShadingHelper::fresnelDielectric(cosThetaI, etaI, etaT);

		float rand = sampler->next();
		Vec3 wi;

		// Reflection
		if (rand < fresnel)
		{
			wi = ShadingHelper::reflect(-wo, normal);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * fresnel;
			pdf = fresnel;
			return wi;
		}
		else // refraction
		{
			float sin2ThetaI = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
			float sin2ThetaT = eta * eta * sin2ThetaI;

			if (sin2ThetaT >= 1.0f)
			{
				//Total Internal Reflection
				wi = ShadingHelper::reflect(-wo, normal);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
				pdf = 1.0f;
				return wi;
			}

			float cosThetaT = std::sqrt(1.0f - sin2ThetaT);
			wi = (-wo) * eta + normal * (eta * cosThetaI - cosThetaT);
			wi = wi.normalize();

			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1.0f - fresnel);
			pdf = 1.0f - fresnel;
			return wi;
		}
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{   
		// Ideal
		return Colour(0.0f, 0.0f, 0.0f);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Ideal
		return 0.0f;
	}

	bool isPureSpecular() { return true; }

	bool isTwoSided() { return false; }

	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

//AS: Add GGX Microfacet BRDF
class GGXBSDF : public BSDF
{
public:
	Texture* albedo;
	float alpha;

	GGXBSDF() = default;

	GGXBSDF(Texture* _albedo, float roughness)
	{
		albedo = _albedo;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wo = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(wo);

		// Importance sample GGX NDF
		float r1 = sampler->next();
		float r2 = sampler->next();
		float theta = atanf(alpha * sqrtf(r1) / sqrtf(1.0f - r1));
		float phi = 2.0f * M_PI * r2;

		float cosTheta = cosf(theta);
		float sinTheta = sinf(theta);

		Vec3 h = Vec3(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta).normalize();

		// reflect vec
		Vec3 wiLocal = -woLocal + h * (2.0f * Dot(woLocal, h));
		if (wiLocal.z <= 0.0f) return Vec3(0.0f, 0.0f, 0.0f); // not use

		Vec3 h_world = shadingData.frame.toWorld(h);
		Vec3 wi_world = shadingData.frame.toWorld(wiLocal);
		Colour albedoSample = albedo->sample(shadingData.tu, shadingData.tv);

		// Fresnel term using Schlick approximation
		float F = ShadingHelper::fresnelDielectric(Dot(wiLocal, h), 1.0f, 1.5f);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		float D = ShadingHelper::Dggx(h, alpha);
		float denom = 4.0f * fabsf(woLocal.z) * fabsf(wiLocal.z);

		reflectedColour = albedoSample * F * (G * D / denom);

		// PDF
		float pdf_h = D * fabsf(h.z) / (4.0f * Dot(woLocal, h));
		pdf = pdf_h;

		return wi_world.normalize();
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{   
		//Transforms both the direction of incidence and the direction of emission from world space to a local coordinate system
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		//lower hemisphere, no sampling
		if (wiLocal.z <= 0.0f || woLocal.z <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);

		Vec3 h = (wiLocal + woLocal).normalize();
		
		//Fresnel
		float F = ShadingHelper::fresnelDielectric(Dot(wiLocal, h), 1.0f, 1.5f);

		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		float D = ShadingHelper::Dggx(h, alpha);
		float denom = 4.0f * fabsf(wiLocal.z) * fabsf(woLocal.z);

		Colour base = albedo->sample(shadingData.tu, shadingData.tv);
		return base * (F * G * D / denom);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{   
		//same as above
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		//lower hemisphere, no sampling
		if (wiLocal.z <= 0.0f || woLocal.z <= 0.0f) return 0.0f;

		Vec3 h = (wiLocal + woLocal).normalize();
		float D = ShadingHelper::Dggx(h, alpha);
		float pdf_h = D * fabsf(h.z) / (4.0f * Dot(woLocal, h));

		return pdf_h;
	}

	bool isPureSpecular() { return false; }

	bool isTwoSided() { return false; }

	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};