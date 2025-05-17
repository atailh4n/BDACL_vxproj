#pragma once
#include "Vec3.hpp"

namespace KGL
{
	struct Ray
	{
		Vec3 origin;
		Vec3 direction;

		Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}

		Vec3 pointAt(float t) const
		{
			return origin + direction * t;
		}
	};
}