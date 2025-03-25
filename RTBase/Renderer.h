#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>


class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter(1, 2));
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour pathTrace(Ray& r, Colour pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			if (depth > MAX_DEPTH) // define in Core.h
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}
			
			Colour bsdf;
			//  Replace cosine sampling in renderer
			Colour indirect;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			if (pdf > 0.0f)
			{
				pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
				r.init(shadingData.x + (wi * EPSILON), wi);
			}
			else
			{
				return direct;
			}
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
			//return computeDirect(shadingData, samplers);
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	//multiple threads and Tile based rendering
	void render() {
		film->incrementSPP();

		const int tileSize = 32;  // Tile 32X32
		const int tilesX = (film->width + tileSize - 1) / tileSize;// render num for X
		const int tilesY = (film->height + tileSize - 1) / tileSize;  //render num for Y
		const int totalTiles = tilesX * tilesY; // all render tiles to map the image

		std::atomic<int> tileCounter(0);  // share tile index job queue

		//Previous multi-threaded architecture in Kurt class
		auto worker = [&](int threadID) {
			Sampler* sampler = &samplers[threadID];
			while (true) {
				int tileIndex = tileCounter.fetch_add(1); 
				if (tileIndex >= totalTiles) break;

				int tileX = tileIndex % tilesX;
				int tileY = tileIndex / tilesX;

				int startX = tileX * tileSize;
				int startY = tileY * tileSize;
				int endX;

				if (startX + tileSize < (int)film->width)
				{
					endX = startX + tileSize;
				}
				else
				{
					endX = (int)film->width;
				}

				int endY;
				if (startY + tileSize < (int)film->height)
				{
					endY = startY + tileSize;
				}
				else
				{
					endY = (int)film->height;
				}

				for (int y = startY; y < endY; y++) {
					for (int x = startX; x < endX; x++) {
						float px = x + 0.5f;
						float py = y + 0.5f;
						Ray ray = scene->camera.generateRay(px, py);

						Colour col = pathTrace(ray, Colour(1.f, 1.f, 1.f), 13, sampler);

						film->splat(px, py, col);

						unsigned char r = (unsigned char)(col.r * 255);
						unsigned char g = (unsigned char)(col.g * 255);
						unsigned char b = (unsigned char)(col.b * 255);
						film->tonemap(x, y, r, g, b);// very improtant!! translate HDR to LDR
						canvas->draw(x, y, r, g, b);
					}
				}
			}
			};

		// using threadPool reduce resource using and speed up.
		std::vector<std::thread> threadPool;
		for (int i = 0; i < numProcs; ++i) {
			threadPool.emplace_back(worker, i);
		}

		// wait all threads over
		for (auto& t : threadPool) {
			t.join();
		}
	}

	//void render()
	//{
	//	film->incrementSPP();
	//	for (unsigned int y = 0; y < film->height; y++)
	//	{
	//		for (unsigned int x = 0; x < film->width; x++)
	//		{
	//			float px = x + 0.5f;
	//			float py = y + 0.5f;
	//			Ray ray = scene->camera.generateRay(px, py);
	//
	//			//Colour col = viewNormals(ray);
	//			//Colour col = albedo(ray);

	//			Colour pathThroughput = Colour(1, 1, 1);
	//			Colour col = pathTrace(ray, pathThroughput, 0, &samplers[0]);

	//			film->splat(px, py, col);
	//			unsigned char r = (unsigned char)(col.r * 255);
	//			unsigned char g = (unsigned char)(col.g * 255);
	//			unsigned char b = (unsigned char)(col.b * 255);
	//			canvas->draw(x, y, r, g, b);
	//		}
	//	}
	//}

	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};