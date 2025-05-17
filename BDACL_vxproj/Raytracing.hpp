#pragma once
#include "Vec3.hpp"
#include "Ray.hpp"
#include "AABB.hpp"
#include "BVHNode.hpp"
#include "HitPoint.hpp"
#include "Triangle.hpp"
#include <memory>
#include <vector>
#include <optional>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace KGL
{
	// Forward declarations
	struct Ray;
	struct HitPoint;
	class BVHNode;

	// Light path tracer (simplified)
	class LightPathTracer
	{
	public:
		LightPathTracer(const std::vector<Triangle>& triangles)
		{
			// Build acceleration structure
			bvh = BVHNode::build(triangles);
		}

		// Trace a single ray through the scene and return closest intersection
		std::optional<HitPoint> traceRay(const Ray& ray) const
		{
			HitPoint hit;
			if (bvh && bvh->intersect(ray, hit))
			{
				return hit;
			}
			return std::nullopt;
		}

		// Trace a light path with multiple bounces
		std::vector<HitPoint> tracePath(const Ray& initialRay, int maxBounces) const
		{
			std::vector<HitPoint> path;
			Ray currentRay = initialRay;

			for (int bounce = 0; bounce < maxBounces; ++bounce)
			{
				auto hit = traceRay(currentRay);
				if (!hit)
					break;

				path.push_back(*hit);

				// Calculate new ray direction (simplified)
				// In a real renderer, this would depend on the material properties
				Vec3 newDir = calculateReflectionDirection(currentRay.direction, hit->normal);
				currentRay = Ray(hit->position + hit->normal * 0.001f, newDir);
			}

			return path;
		}

	private:
		std::unique_ptr<BVHNode> bvh;

		// Calculate reflection direction (simplified)
		Vec3 calculateReflectionDirection(const Vec3& incident, const Vec3& normal) const
		{
			return incident - normal * 2.0f * incident.dot(normal);
		}
	};
}