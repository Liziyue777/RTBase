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
		float t = acos(1.0f-r1);
		return SphericalCoordinates::sphericalToWorld(t, p);

	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		return 1.0f/(2.0f*M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		float r = sqrtf(r1);
		float t = 2.0f * M_PI * r2;
		float x = r * cosf(t);
		float y = r * sinf(t);
		float z = sqrtf(1.0f - x * x - y * y);
		return Vec3(x, y, z);
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
		float t = acos(1.0f - 2.0f * r1);
		return SphericalCoordinates::sphericalToWorld(t,p);
	}

	static float uniformSpherePDF(const Vec3& wi)
	{
		return 1.0f/(4.0f*M_PI);
	}
};