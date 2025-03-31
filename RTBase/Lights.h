#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		//! Area light!
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
	    
		// Using cosine sampling for the arealight
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wiLocal);

		// Local coordinate system
		Frame frame;
		frame.fromVector(triangle->gNormal());

		// translate to world space
		return frame.toWorld(wiLocal);
		
		//old version(provide)
       /*Vec3 wi = Vec3(0, 0, 1);
		pdf = 1.0f;
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);*/
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		//!Environment light
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;

	std::vector<float> pdf2D;   
	std::vector<float> cdf2D;       
	std::vector<float> marginalPdf; 
	std::vector<float> marginalCdf; 

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		pdf2D.resize(env->width * env->height);
		cdf2D.resize(env->width * env->height);
		marginalPdf.resize(env->height);
		marginalCdf.resize(env->height);

		// Compute PDF for each texel
		float sum = 0.0f;
		for (int y = 0; y < env->height; ++y) {
			float theta = M_PI * (y + 0.5f) / (float)env->height;
			float sinTheta = sinf(theta);
			for (int x = 0; x < env->width; ++x) {
				int index = y * env->width + x;
				float lum = env->texels[index].Lum();
				float weight = lum * sinTheta;
				pdf2D[index] = weight;
				sum += weight;
			}
		}

		// Normalize PDF
		for (float& p : pdf2D) p /= sum;

		// Compute CDF (column-wise)
		for (int y = 0; y < env->height; ++y) {
			float rowSum = 0.0f;
			for (int x = 0; x < env->width; ++x) {
				int idx = y * env->width + x;
				rowSum += pdf2D[idx];
				cdf2D[idx] = rowSum;
			}
			marginalPdf[y] = rowSum;

			// Normalize the current row CDF to [0, 1].
			if (rowSum > 0.0f) {
				for (int x = 0; x < env->width; ++x) {
					int idx = y * env->width + x;
					cdf2D[idx] /= rowSum;
				}
			}
		}
		// Normalize marginal PDF
		float marginalSum = 0.0f;
		for (float m : marginalPdf) marginalSum += m;
		for (float& m : marginalPdf) m /= marginalSum;

		// Compute marginal CDF
		float cdfSum = 0.0f;
		for (int y = 0; y < env->height; ++y) {
			cdfSum += marginalPdf[y];
			marginalCdf[y] = cdfSum;
		}
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel (done)
		
		Vec3 wi = sampleDirectionFromLight(sampler, pdf); // Luminance-weighted importance sampling (implement see below)
		reflectedColour = evaluate(shadingData, wi);
		return wi;
		
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;*/
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		if (u < 0.0f) {
			u = u + 2.0f * M_PI;
		}
		u = u / (2.0f * M_PI);

		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling (done)
		// 
		float u = atan2f(wi.z, wi.x);
		if (u < 0.0f)
		{
			u = u + 2.0f * M_PI;
		}
		u /= 2.0f * M_PI;
		float v = acosf(wi.y) / M_PI;

		int x = std::min((int)(u * env->width), env->width - 1);
		int y = std::min((int)(v * env->height), env->height - 1);
		int idx = y * env->width + x;

		float theta = (float)(y + 0.5f) * M_PI / env->height;
		float sinTheta = sinf(theta);

		return pdf2D[idx] * env->width * env->height / (2.0f * M_PI * M_PI * sinTheta + EPSILON);
		
		//return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps (done)
		
		float u = sampler->next();
		float v = sampler->next();

		// Sample y index from marginal CDF
		int y = std::lower_bound(marginalCdf.begin(), marginalCdf.end(), v) - marginalCdf.begin();

		// Sample x index from conditional CDF
		int rowStart = y * env->width;
		int x = std::lower_bound(cdf2D.begin() + rowStart, cdf2D.begin() + rowStart + env->width, u) - (cdf2D.begin() + rowStart);

		// Convert to spherical direction
		float uCoord = (x + 0.5f) / env->width;
		float vCoord = (y + 0.5f) / env->height;
		float theta = vCoord * M_PI;
		float phi = uCoord * 2.0f * M_PI;

		float sinTheta = sinf(theta);
		Vec3 wi(
			sinTheta * cosf(phi),
			cosf(theta),
			sinTheta * sinf(phi)
		);

		// Compute PDF
		pdf = pdf2D[y * env->width + x] * env->width * env->height / (2.0f * M_PI * M_PI * sinTheta + EPSILON);

		return wi;

		
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;*/
	}
};