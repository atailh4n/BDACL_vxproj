#pragma once
#include "AStar.hpp"
#include "DAGL.hpp"
#include "SIMT.hpp"
#include "Raytracing.hpp"
#include <chrono>
#include <random>
#include <mutex>
#include <string>
#include <iostream>
#include <vector>
#include <future>
#include <memory>

using namespace KGL;

namespace BenchFW {
	static void RayIntersectsTriangleSimulation_Scalable(int triangleCount = 10000, int rayCount = 1000) {
		std::vector<Triangle> triangles;
		std::mutex coutMutex;
		BDACL::ThreadSafeLogger logger;
		triangles.reserve(triangleCount);

		std::mt19937 rng(std::random_device{}());
		std::uniform_real_distribution<float> dist(-100.0f, 100.0f);

		// Rastgele üçgenler üret
		for (int i = 0; i < triangleCount; ++i) {
			Vec3 v0(dist(rng), dist(rng), dist(rng));
			Vec3 v1 = v0 + Vec3(dist(rng) * 0.1f, dist(rng) * 0.1f, dist(rng) * 0.1f);
			Vec3 v2 = v0 + Vec3(dist(rng) * 0.1f, dist(rng) * 0.1f, dist(rng) * 0.1f);
			triangles.emplace_back(v0, v1, v2);
		}

		// BVH oluştur
		auto bvhRoot = BVHNode::build(triangles);

		// Işın üret
		std::vector<Ray> rays;
		rays.reserve(rayCount);
		for (int i = 0; i < rayCount; ++i) {
			Vec3 origin(dist(rng), dist(rng), dist(rng));
			Vec3 dir(dist(rng), dist(rng), dist(rng));
			rays.emplace_back(origin, dir.normalize());
		}

		// Çarpışma testi
		int hitCount = 0;
		for (const auto& ray : rays) {
			HitPoint hit;
			std::function<bool(const BVHNode*, const Ray&, HitPoint&)> traverse;
			traverse = [&](const BVHNode* node, const Ray& ray, HitPoint& hit) -> bool {
				if (!node) return false;
				float tMin, tMax;
				if (!node->bounds.intersect(ray, tMin, tMax)) return false;

				bool hitSomething = false;
				if (node->isLeaf) {
					for (const auto& tri : node->triangles) {
						if (tri.intersect(ray, hit)) {
							hitSomething = true;
						}
					}
				}
				else {
					hitSomething |= traverse(node->left.get(), ray, hit);
					hitSomething |= traverse(node->right.get(), ray, hit);
				}
				return hitSomething;
				};

			if (traverse(bvhRoot.get(), ray, hit)) {
				++hitCount;
			}
		}
		std::string text = "[RT] Rays: " + std::to_string(rayCount) + ", hits: " + std::to_string(hitCount);
		logger.log(text);
	}

	static void AStarPathfinderTest_Scalable(int nodeCount = 1000, int connectionsPerNode = 3) {
		AStar<int> astar;
		std::unordered_map<int, std::vector<std::pair<int, float>>> graph;
		std::unordered_map<int, std::pair<float, float>> positions; // x, y pozisyonları

		std::mt19937 rng(std::random_device{}());
		std::uniform_real_distribution<float> posDist(0.0f, 1000.0f);
		std::uniform_int_distribution<int> nodeDist(0, nodeCount - 1);

		// Pozisyonlar ve bağlantılar
		for (int i = 0; i < nodeCount; ++i) {
			positions[i] = { posDist(rng), posDist(rng) };
		}

		for (int i = 0; i < nodeCount; ++i) {
			for (int j = 0; j < connectionsPerNode; ++j) {
				int neighbor = nodeDist(rng);
				if (neighbor != i) {
					float dx = positions[i].first - positions[neighbor].first;
					float dy = positions[i].second - positions[neighbor].second;
					float cost = std::sqrt(dx * dx + dy * dy);
					graph[i].emplace_back(neighbor, cost);
				}
			}
		}

		auto getNeighbors = [&](const int& node) -> std::vector<std::pair<int, float>> {
			return graph[node];
			};

		auto heuristic = [&](const int& from, const int& to) -> float {
			auto [x1, y1] = positions[from];
			auto [x2, y2] = positions[to];
			float dx = x1 - x2;
			float dy = y1 - y2;
			return std::sqrt(dx * dx + dy * dy);
			};

		int start = nodeDist(rng);
		int goal = nodeDist(rng);

		auto path = astar.findPath(start, goal, getNeighbors, heuristic);

		//std::cout << "[AStar]: " << start << ", Dest: " << goal << ", Loftr: " << path.value().size() << "\n";
	}

#pragma region SIMTvsBDACL
	static long long runDATaskSchedulerBenchmark(int threads) {
		auto start_bdacl = std::chrono::high_resolution_clock::now();
		BDACL::DATask_Scheduler scheduler;
		const int TASK_COUNT = threads;

		std::vector<BDACL::DATask> tasks(TASK_COUNT);
		std::vector<std::future<void>> futures;

		for (int i = 0; i < TASK_COUNT; ++i) {
			auto promise = std::make_shared<std::promise<void>>();
			futures.push_back(promise->get_future());
			tasks[i].job = [promise] {
				RayIntersectsTriangleSimulation_Scalable(20'000, 4'000);
				promise->set_value();
				};
			tasks[i].jobPendingDependencies = 0;
		}

		// SADECE 1 DEFA çağır
		scheduler.runAwaitingTasks(threads);

		// Tüm task'ları sırayla ekle
		for (auto& task : tasks) {
			scheduler.addTask(&task);
		}

		for (auto& fut : futures) {
			fut.get();
		}

		auto end_bdacl = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::microseconds>(end_bdacl - start_bdacl).count();
	}


	static long long runSIMTBenchmark(int threads) {
		std::vector<std::function<void(int)>> program = {
			[](int id) {
				RayIntersectsTriangleSimulation_Scalable(20000, 4000);
			}
		};
		auto start_simt = std::chrono::high_resolution_clock::now();
		SIMT::Constructor simt(threads, program);
		simt.execute(threads);
		auto end_simt = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::microseconds>(end_simt - start_simt).count();
	}

#pragma endregion

}