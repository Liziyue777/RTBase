#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		
		float p = 2.0f * M_PI * r2;
		float t = acos(r1);
		float PDF = 1 / (4*M_PI);
		float CDFtheta = (1 - cos(t))/2;
		float CDFphi = p / (2 * M_PI);
		return SphericalCoordinates::sphericalToWorld(t, p);

	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		return 1/(2*M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		float t = acosf(sqrtf(r1));
		float p = 2.0f * M_PI * r2;
		return Vec3(0, 0, 1);
	}

	static float cosineHemispherePDF(const Vec3 wi,float r1)
	{
		float t = acosf(sqrtf(r1));
		return cosf(t)/M_PI;
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		return wi.z / M_PI;
	}

	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		float p = 2.0f * M_PI * r2;
		float t = acos(1 - 2 * r1);
		return SphericalCoordinates::sphericalToWorld(t,p);
	}

	static float uniformSpherePDF(const Vec3& wi)
	{
		return 1/(4*M_PI);
	}
};