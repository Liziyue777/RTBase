﻿#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(float x, float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const override
	{
		return (fabsf(x) <= 0.5f && fabsf(y) <= 0.5f) ? 1.0f : 0.0f;
	}
	int size() const override { return 0; }
};



// New work:
// GaussianFilter
float Gaussians(float alpha, float x)
{
	return expf(-alpha * x * x);
}

class GaussianFilter : public ImageFilter
{
public:
	float radius, alpha;
	GaussianFilter(float r, float a) : radius(r), alpha(a) {}

	float filter(float x, float y) const override
	{
		if (fabsf(x) > 0.5f || fabsf(y) > 0.5f) return 0.0f;
		return Gaussians(alpha, x) * Gaussians(alpha, y);
	}

	int size() const override
	{
		return (int)ceilf(radius) * 2;
	}
};

//Mitchell-Netravali filter
class MitchellNetravaliFilter : public ImageFilter
{
private:
	float B, C;

public:
	MitchellNetravaliFilter(float _B = 1.0f / 3.0f, float _C = 1.0f / 3.0f)
	{
		B = _B;
		C = _C;
	}

	int size() const override
	{
		return 2; // Filter support: [-2, 2]
	}

	float mitchell1D(float x) const
	{
		x = fabsf(x);
		if (x < 1.0f)
		{
			return ((12 - 9 * B - 6 * C) * x * x * x +(-18 + 12 * B + 6 * C) * x * x +(6 - 2 * B)) / 6.0f;
		}
		else if (x < 2.0f)
		{
			return ((-B - 6 * C) * x * x * x +(6 * B + 30 * C) * x * x +(-12 * B - 48 * C) * x +(8 * B + 24 * C)) / 6.0f;
		}
		else
		{
			return 0.0f;
		}
	}

	// 2D  separable filter
	float filter(float x, float y) const override
	{
		return mitchell1D(x) * mitchell1D(y);
	}
};

class Film
{
public:
	Colour* film;
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;

	void splat(const float x, const float y, const Colour& L)
	{
		float filterWeights[25];    // // Storage to cache weights 
		unsigned int indices[25];   // // Store indices to minimize computations 
		unsigned int used = 0;
		float total = 0.0f;
		int size = filter->size(); // radius

		for (int i = -size; i <= size; ++i)
		{
			for (int j = -size; j <= size; ++j)
			{
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < (int)width && py >= 0 && py < (int)height)
				{
					unsigned int index = py * width + px;
					float weight = filter->filter((float)j, (float)i);
					filterWeights[used] = weight;
					indices[used] = index;
					total += weight;
					used++;
				}
			}
		}

		for (unsigned int i = 0; i < used; ++i)
		{
			film[indices[i]] = film[indices[i]] + (L * (filterWeights[i] / total));
		}
	}

	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		//get HDR color
		Colour hdrColor = film[y * width + x] * exposure / (float)SPP;

		// (sRGB gamma = 2.2)
		hdrColor.r = powf(hdrColor.r, 1.0f / 2.2f);
		hdrColor.g = powf(hdrColor.g, 1.0f / 2.2f);
		hdrColor.b = powf(hdrColor.b, 1.0f / 2.2f);

		// reflect 0-255
		r = (std::min(255.0f, std::max(0.0f, hdrColor.r * 255.0f)));
		g = (std::min(255.0f, std::max(0.0f, hdrColor.g * 255.0f)));
		b = (std::min(255.0f, std::max(0.0f, hdrColor.b * 255.0f)));
	}


	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		clear();
		filter = _filter;
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};