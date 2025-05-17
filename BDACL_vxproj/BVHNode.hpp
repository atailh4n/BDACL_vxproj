#pragma once
#include "Vec3.hpp"
#include "Ray.hpp"
#include "AABB.hpp"
#include "HitPoint.hpp"
#include "Triangle.hpp"
#include "BVHNode.hpp"
#include <memory>
#include <vector>
#include <algorithm>

namespace KGL
{
    class BVHNode {
    public:
        AABB bounds;
        std::unique_ptr<BVHNode> left;
        std::unique_ptr<BVHNode> right;
        std::vector<Triangle> triangles;
        bool isLeaf;

        BVHNode() : isLeaf(false) {}

        // Build BVH from a list of triangles
        static std::unique_ptr<BVHNode> build(std::vector<Triangle> triangles, int maxTrianglesPerLeaf = 4)
        {
            if (triangles.empty())
            {
                return nullptr;
            }

            auto node = std::make_unique<BVHNode>();

            // Calculate bounding box
            node->bounds = triangles[0].getBounds();
            for (size_t i = 1; i < triangles.size(); ++i)
            {
                node->bounds = node->bounds.merge(triangles[i].getBounds());
            }

            // If few enough triangles, make this a leaf
            if (triangles.size() <= static_cast<size_t>(maxTrianglesPerLeaf))
            {
                node->triangles = std::move(triangles);
                node->isLeaf = true;
                return node;
            }

            // Find axis with greatest extent
            Vec3 extent = node->bounds.max - node->bounds.min;
            int axis = 0;
            if (extent.y > extent.x)
                axis = 1;
            if (extent.z > extent[axis])
                axis = 2;

            // Sort triangles along chosen axis
            std::sort(triangles.begin(), triangles.end(), [axis](const Triangle& a, const Triangle& b)
                { return a.centroid()[axis] < b.centroid()[axis]; });

            // Split triangles into two groups
            size_t mid = triangles.size() / 2;
            std::vector<Triangle> leftTris(triangles.begin(), triangles.begin() + mid);
            std::vector<Triangle> rightTris(triangles.begin() + mid, triangles.end());

            // Recursively build children
            node->left = build(std::move(leftTris), maxTrianglesPerLeaf);
            node->right = build(std::move(rightTris), maxTrianglesPerLeaf);

            return node;
        }

        // Intersect ray with BVH
        bool intersect(const Ray& ray, HitPoint& hit) const
        {
            float tMin, tMax;
            if (!bounds.intersect(ray, tMin, tMax))
            {
                return false;
            }

            // If we're a leaf, check all triangles
            if (isLeaf)
            {
                bool hitAnything = false;
                for (const auto& triangle : triangles)
                {
                    if (triangle.intersect(ray, hit))
                    {
                        hitAnything = true;
                    }
                }
                return hitAnything;
            }

            // Otherwise check children
            bool hitLeft = left && left->intersect(ray, hit);
            bool hitRight = right && right->intersect(ray, hit);

            return hitLeft || hitRight;
        }
    };
}