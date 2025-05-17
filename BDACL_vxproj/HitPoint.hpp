#pragma once
#include "Vec3.hpp"

namespace KGL
{
	struct HitPoint {
		float t;          // Distance along ray
		Vec3 position;    // Position of hit
		Vec3 normal;      // Surface normal
		int materialId;   // Material identifier

		HitPoint() : t(std::numeric_limits<float>::max()), materialId(-1) {}
	};
} // namespace GPUJobs
