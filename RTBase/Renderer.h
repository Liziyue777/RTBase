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

#include <OpenImageDenoise/oidn.hpp> 


// Used in the Instant Radiosity algorithm to simulate lighting.
struct VirtualPointLight {
	Vec3 position;
	Vec3 normal; //calculate G
	Colour power; //radiant power
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;

	//Image storage(temporary) for OIDN
	std::vector<float> colorBuffer;
	std::vector<float> albedoBuffer;
	std::vector<float> normalBuffer;

	std::vector<VirtualPointLight> vplList; //store all VPL

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
		colorBuffer.resize(film->width * film->height * 3);
		albedoBuffer.resize(film->width * film->height * 3);
		normalBuffer.resize(film->width * film->height * 3);
		clear();
	}
	void clear()
	{
		film->clear();
	}

	void buildVPLs(int numVPLs = 1000)//here shoot 1000 path to create VPL
	{   
		vplList.clear(); // clear VPL list before to create new
		
		//Each path simulates a ray of light from a light source
		for (int i = 0; i < numVPLs; ++i)
		{
			Sampler* sampler = &samplers[i % numProcs]; // for multiple threads 
			float lightPmf;
			Light* light = scene->sampleLight(sampler, lightPmf); // get the light for scene
			
			// Sample emission point/emission direction
			float pdfPos, pdfDir;
			Vec3 lightPos = light->samplePositionFromLight(sampler, pdfPos);
			Vec3 lightDir = light->sampleDirectionFromLight(sampler, pdfDir);
			
			// get the real light from sample in light.h
			Colour emitted;
			float dummyPdf;
			light->sample(ShadingData(), sampler, emitted, dummyPdf);

			Ray r(lightPos, lightDir);// shoot 
			IntersectionData isect = scene->traverse(r);
			if (isect.t < FLT_MAX)
			{
				ShadingData shadingData = scene->calculateShadingData(isect, r);
				if (!shadingData.bsdf->isPureSpecular()) // If the surface is mirrored, no VPL 
				{
					Colour f = shadingData.bsdf->evaluate(shadingData, -lightDir);
					float bsdfPdf = shadingData.bsdf->PDF(shadingData, -lightDir);
					Colour power = emitted * f / (pdfPos * pdfDir + EPSILON);  //Calculate the power of the VPL.

					vplList.push_back({ shadingData.x, shadingData.sNormal, power }); // store light
				}
			}
		}
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular())
			return Colour(0.0f, 0.0f, 0.0f);

		Colour Ld(0.0f, 0.0f, 0.0f);

		// Light Sampling
		float lightPmf;
		Light* light = scene->sampleLight(sampler, lightPmf);
		float lightPdf;
		Colour emitted;
		Vec3 sample = light->sample(shadingData, sampler, emitted, lightPdf);

		Vec3 wi;
		float GTerm = 1.0f;

		if (light->isArea())
		{
			wi = (sample - shadingData.x).normalize();
			float lenSq = (sample - shadingData.x).lengthSq();
			float cosThetaSurface = max(0.0f, Dot(wi, shadingData.sNormal));
			float cosThetaLight = max(0.0f, -Dot(wi, light->normal(shadingData, wi)));
			GTerm = (cosThetaSurface * cosThetaLight) / (lenSq + EPSILON);
		}
		else
		{
			wi = sample; // already a direction
			GTerm = max(0.0f, Dot(shadingData.sNormal, wi));
		}

		if (GTerm > 0 && scene->visible(shadingData.x, light->isArea() ? sample : shadingData.x + wi * 1e4f))
		{
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			float bsdfPdf = shadingData.bsdf->PDF(shadingData, wi);
			float weight = (lightPdf * lightPdf) / (lightPdf * lightPdf + bsdfPdf * bsdfPdf + EPSILON);
			Ld = Ld + f * emitted * GTerm * weight / (lightPdf * lightPmf + EPSILON);
		}

		// BSDF Sampling
		float bsdfPdf;
		Colour fBSDF;
		Vec3 bsdfDir = shadingData.bsdf->sample(shadingData, sampler, fBSDF, bsdfPdf);

		if (fBSDF.Lum() > 0.0f && bsdfPdf > 0.0f)
		{
			Vec3 rayTarget = shadingData.x + bsdfDir * 1e4f;
			if (scene->visible(shadingData.x, rayTarget))
			{
				for (Light* l : scene->lights)
				{
					Colour Le = l->evaluate(shadingData, bsdfDir);
					if (Le.Lum() > 0.0f)
					{
						float lightPdf2 = l->PDF(shadingData, bsdfDir);
						float cosTheta = max(0.0f, Dot(shadingData.sNormal, bsdfDir));
						float weight = (bsdfPdf * bsdfPdf) / (bsdfPdf * bsdfPdf + lightPdf2 * lightPdf2 + EPSILON);
						Ld = Ld + fBSDF * Le * cosTheta * weight / (bsdfPdf + EPSILON);
						break; // break when it reach a light
					}
				}
			}
		}

		return Ld;
	}

	//This function is build to calculate the total indirect lighting received at the shading point from all VPL.
	Colour computeVPL(const ShadingData& shadingData)
	{
		Colour indirect(0.0f, 0.0f, 0.0f);
		for (const VirtualPointLight& vpl : vplList)
		{
			Vec3 wi = (vpl.position - shadingData.x).normalize();
			
			//Calculate variables
			float lenSq = (vpl.position - shadingData.x).lengthSq();
			float G = max(0.0f, Dot(shadingData.sNormal, wi)) *max(0.0f, Dot(vpl.normal, -wi)) /(lenSq + EPSILON);
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			
			//power × BSDF × GTerm
			indirect = indirect + f * vpl.power * G;
		}
		return indirect;
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
			//Combine two methods
			Colour Ld = computeDirect(shadingData, sampler);
			Colour Li = computeVPL(shadingData);
			return Ld + Li;
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

						//Colour col = pathTrace(ray, Colour(1.f, 1.f, 1.f), 13, sampler);
						Colour col = direct(ray, sampler);
						
						
						film->splat(px, py, col);
						// renderer Albedo and Normal
						Colour alb = albedo(ray);
						Colour nrm = viewNormals(ray);

						// To the AOV buffer
						int index = (y * film->width + x) * 3;
						colorBuffer[index + 0] = col.r;
						colorBuffer[index + 1] = col.g;
						colorBuffer[index + 2] = col.b;

						albedoBuffer[index + 0] = alb.r;
						albedoBuffer[index + 1] = alb.g;
						albedoBuffer[index + 2] = alb.b;

						normalBuffer[index + 0] = nrm.r;
						normalBuffer[index + 1] = nrm.g;
						normalBuffer[index + 2] = nrm.b;

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

		// call OIDN (below from PPT)
		oidn::DeviceRef device = oidn::newDevice(); // The CPU can not be found although l add OIDN path to VS and download API runtime in my cp.
		device.commit();

		oidn::FilterRef filter = device.newFilter("RT");
		filter.setImage("color", colorBuffer.data(), oidn::Format::Float3, film->width, film->height);
		filter.setImage("albedo", albedoBuffer.data(), oidn::Format::Float3, film->width, film->height);
		filter.setImage("normal", normalBuffer.data(), oidn::Format::Float3, film->width, film->height);
		std::vector<float> output(film->width * film->height * 3);
		filter.setImage("output", output.data(), oidn::Format::Float3, film->width, film->height);
		filter.set("hdr", true);
		filter.commit();
		filter.execute();

		// Here I've added a judgment that if the configured environment is OK (CPU can be found) then noise reduction will be applied
		bool oidnSuccess = false;
		const char* errorMessage;
		auto err = device.getError(errorMessage);
		if (err == oidn::Error::None)
		{
			std::cout << "OIDN used successfully" << std::endl; //show if the OIDN apply
			oidnSuccess = true;
		}
		else
		{
			//Here will fallback to original non-denoised rendering
		}


		if (oidnSuccess)
		{   // The denoised image will cover the rendered result
			for (int y = 0; y < film->height; ++y)
			{
				for (int x = 0; x < film->width; ++x)
				{
					int index = (y * film->width + x) * 3;
					unsigned char r = (unsigned char)(((output[index + 0] < 1.0f) ? output[index + 0] : 1.0f) * 255.0f);
					unsigned char g = (unsigned char)(((output[index + 1] < 1.0f) ? output[index + 1] : 1.0f) * 255.0f);
					unsigned char b = (unsigned char)(((output[index + 2] < 1.0f) ? output[index + 2] : 1.0f) * 255.0f);

					canvas->draw(x, y, r, g, b);
				}
			}
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