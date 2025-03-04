#pragma once
#define EPSILON 0.001f
#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		float denom = Dot(n, r.dir); // 计算 N·D

		// 如果 denom 近似为 0，则光线平行于平面，无交点
		if (fabs(denom) < EPSILON)
			return false;

		// 计算交点参数 t
		t = -(Dot(n, r.o) + d) / denom;

		// 交点在光线的负方向（t < 0），则无交点
		if (t < 0)
			return false;

		// 其余情况光线与平面相交
		return true;
	}
};


class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
        // Möller-Trumbore
		Vec3 E1 = vertices[1].p - vertices[0].p;
		Vec3 E2 = vertices[2].p - vertices[0].p;

		// 计算 P 向量 (P = D × E2)
		Vec3 P = r.dir.cross(E2);

		// 计算行列式 determinant (det = E1 · P)
		float det = Dot(E1, P);

		// 光线与三角形平行，直接返回 false
		if (fabs(det) < EPSILON) return false;

		float invDet = 1.0f / det; //store invdet = 1/det

		// 计算 T 向量 (T = O - V0)
		Vec3 T = r.o - vertices[0].p;

		// 计算重心坐标 u (u = (P · T) * invDet)
		u = Dot(P, T) * invDet;

		// u 超出范围，返回 false
		if (u < 0.0f || u > 1.0f) return false;

		// 计算 Q 向量 (Q = T × E1)
		Vec3 Q = T.cross(E1);

		// 计算重心坐标 v (v = (D · Q) * invDet)
		v = Dot(Q, r.dir) * invDet;

		// v 超出范围，返回 false
		if (v < 0.0f || (u + v) > 1.0f) return false;

		// 计算 t (t = (E2 · Q) * invDet)
		t = Dot(Q, E2) * invDet;

		// 只有当 t > 0，才表示光线射向三角形
		return (t > EPSILON);

		// Toms code
		// 
		//float denom = Dot(n, r.dir);
		//if (denom == 0) { return false; }
		//t = (d - Dot(n, r.o)) / denom;
		//if (t < 0) { return false; }
		//Vec3 p = r.at(t);
		//float invArea = 1.0f / Dot(e1.cross(e2), n);
		//u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		//if (u < 0 || u > 1.0f) { return false; }
		//v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		//if (v < 0 || (u + v) > 1.0f) { return false; }
		//return true;
	}



	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		return Vec3(0, 0, 0);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{    
		//X-axis
		float tmin_x = (min.x - r.o.x) * r.invDir.x;
		float tmax_x = (max.x - r.o.x) * r.invDir.x;
		if (tmin_x > tmax_x) std::swap(tmin_x, tmax_x);

		//Y-axis
		float tmin_y = (min.y - r.o.y) * r.invDir.y;
		float tmax_y = (max.y - r.o.y) * r.invDir.y;
		if (tmin_y > tmax_y) std::swap(tmin_y, tmax_y);

		//Z-axis
		float tmin_z = (min.z - r.o.z) * r.invDir.z;
		float tmax_z = (max.z - r.o.z) * r.invDir.z;
		if (tmin_z > tmax_z) std::swap(tmin_z, tmax_z);

		float tmin = std::max({ tmin_x, tmin_y, tmin_z });
		float tmax = std::min({ tmax_x, tmax_y, tmax_z });

		// If tmin > tmax, the ray misses the AABB; if tmax < 0,the intersection is before the starting point of the ray
		if (tmin > tmax || tmax < 0) return false;

	    t = tmin; // intersection t point,the ray intersects the AABB
		return true;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		float t;
		return rayAABB(r, t);
	}

	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		// 计算光线起点到球心的向量 L = O - C
		Vec3 L = r.o - centre;

		// 计算 A, B, C（三个系数）
		float A = Dot(r.dir, r.dir);  // 通常等于 1
		float B = 2.0f * Dot(r.dir, L);
		float C = Dot(L, L) - radius * radius;

		// 计算判别式 Δ = B^2 - 4AC
		float delta = B * B - 4 * A * C;

		// 没有交点
		if (delta < 0) return false;

		// 计算两个交点 t1 和 t2
		float sqrt_delta = sqrt(delta);
		float t1 = (-B - sqrt_delta) / (2.0f * A);
		float t2 = (-B + sqrt_delta) / (2.0f * A);

		// 选择 t
		if (t1 > 0) {
			t = t1;
			return true;
		}
		else if (t2 > 0) {
			t = t2;
			return true;
		}

		return false; // 光线在球体内部，但朝向外部，不计算交点
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		// Add BVH building code here
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		return true;
	}
};
