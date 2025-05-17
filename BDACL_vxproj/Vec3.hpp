#pragma once
#include <stdexcept>
#include <cmath>

namespace KGL
{
	struct Vec3
	{
		float x, y, z;
		float operator[](int index) const
		{
			switch (index)
			{
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
			default:
				throw std::out_of_range("Index must be 0, 1 or 2");
			}
		}

		Vec3() : x(0), y(0), z(0) {}
		Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

		Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
		Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
		Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }

		float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
		Vec3 cross(const Vec3& v) const
		{
			return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
		}

		float length() const { return std::sqrt(x * x + y * y + z * z); }
		Vec3 normalize() const
		{
			float len = length();
			if (len > 0)
				return Vec3(x / len, y / len, z / len);
			return *this;
		}
	};
}