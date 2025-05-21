#pragma once
#include "Vec3.hpp"
#include "Ray.hpp"
#include "AABB.hpp"

namespace KGL
{
	// Abstract triangle representation
	class Triangle {
	public:
		Vec3 v0, v1, v2;
		Vec3 normal;
		int materialId;
		Triangle() = default;
		Triangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, int matId = 0)
			: v0(v0), v1(v1), v2(v2), materialId(matId) {
			normal = (v1 - v0).cross(v2 - v0).normalize();
		}

		bool intersect(const Ray& ray, HitPoint& hit) const {
			const float EPSILON = 0.0000001f;

			// Calculate edges
			Vec3 edge1 = v1 - v0;
			Vec3 edge2 = v2 - v0;

			// Calculate determinant
			Vec3 h = ray.direction.cross(edge2);
			float a = edge1.dot(h);

			// Check if ray is parallel to triangle
			if (a > -EPSILON && a < EPSILON) return false;

			float f = 1.0f / a;
			Vec3 s = ray.origin - v0;
			float u = f * s.dot(h);

			// Check if hit point is outside triangle
			if (u < 0.0f || u > 1.0f) return false;

			Vec3 q = s.cross(edge1);
			float v = f * ray.direction.dot(q);

			// Check if hit point is outside triangle
			if (v < 0.0f || u + v > 1.0f) return false;

			// Calculate distance along ray
			float t = f * edge2.dot(q);

			// Check if intersection is behind ray origin
			if (t < EPSILON) return false;

			// We have a valid intersection
			if (t < hit.t) {
				hit.t = t;
				hit.position = ray.pointAt(t);
				hit.normal = normal;
				hit.materialId = materialId;
				return true;
			}

			return false;
		}

		AABB getBounds() const {
			return AABB(
				Vec3(std::min({ v0.x, v1.x, v2.x }),
					std::min({ v0.y, v1.y, v2.y }),
					std::min({ v0.z, v1.z, v2.z })),
				Vec3(std::max({ v0.x, v1.x, v2.x }),
					std::max({ v0.y, v1.y, v2.y }),
					std::max({ v0.z, v1.z, v2.z }))
			);
		}

		Vec3 centroid() const {
			return Vec3(
				(v0.x + v1.x + v2.x) / 3.0f,
				(v0.y + v1.y + v2.y) / 3.0f,
				(v0.z + v1.z + v2.z) / 3.0f
			);
		}
	};
} // namespace GPUJobs