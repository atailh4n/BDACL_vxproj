#pragma once
#include "Vec3.hpp"
#include "Ray.hpp"

namespace KGL
{
	// Axis-aligned bounding box
	struct AABB
	{
		Vec3 min;
		Vec3 max;

		AABB() : min(Vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max())),
			max(Vec3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()))
		{
		}

		AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {}

		bool intersect(const Ray& ray, float& tMin, float& tMax) const
		{
			float txMin = (min.x - ray.origin.x) / ray.direction.x;
			float txMax = (max.x - ray.origin.x) / ray.direction.x;

			if (txMin > txMax)
				std::swap(txMin, txMax);

			float tyMin = (min.y - ray.origin.y) / ray.direction.y;
			float tyMax = (max.y - ray.origin.y) / ray.direction.y;

			if (tyMin > tyMax)
				std::swap(tyMin, tyMax);

			if (txMin > tyMax || tyMin > txMax)
				return false;

			if (tyMin > txMin)
				txMin = tyMin;
			if (tyMax < txMax)
				txMax = tyMax;

			float tzMin = (min.z - ray.origin.z) / ray.direction.z;
			float tzMax = (max.z - ray.origin.z) / ray.direction.z;

			if (tzMin > tzMax)
				std::swap(tzMin, tzMax);

			if (txMin > tzMax || tzMin > txMax)
				return false;

			if (tzMin > txMin)
				txMin = tzMin;
			if (tzMax < txMax)
				txMax = tzMax;

			tMin = txMin;
			tMax = txMax;

			return txMax > 0;
		}

		AABB merge(const AABB& other) const
		{
			return AABB(
				Vec3(std::min(min.x, other.min.x),
					std::min(min.y, other.min.y),
					std::min(min.z, other.min.z)),
				Vec3(std::max(max.x, other.max.x),
					std::max(max.y, other.max.y),
					std::max(max.z, other.max.z)));
		}

		Vec3 centroid() const
		{
			return Vec3(
				(min.x + max.x) * 0.5f,
				(min.y + max.y) * 0.5f,
				(min.z + max.z) * 0.5f);
		}
	};
}