#pragma once
#include "AStar.hpp"
#include "BDACL.hpp"
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
#include <omp.h>

static constexpr int THREADS = 8;
static constexpr int RAYC = 5'000;
static constexpr int TRIGC = 50'000;

using namespace KGL;

namespace BenchFW {
	static void RayIntersectsTriangleSimulation_Scalable(int triangleCount = TRIGC, int rayCount = RAYC) {
		std::vector<Triangle> triangles;
		std::mutex coutMutex;
		//BDACL::ThreadSafeLogger logger;
		triangles.reserve(triangleCount);

		std::mt19937 rng(std::random_device{}());
		std::uniform_real_distribution<float> dist(-100.0f, 100.0f);

		// Rastgele üçgenler üret
		#pragma omp parallel for
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
		#pragma omp parallel for
		for (int i = 0; i < rayCount; ++i) {
			Vec3 origin(dist(rng), dist(rng), dist(rng));
			Vec3 dir(dist(rng), dist(rng), dist(rng));
			rays.emplace_back(origin, dir.normalize());
		}

		// Çarpışma testi
		int hitCount = 0;
		#pragma omp parallel for
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

		//std::string text = "[RT] Rays: " + std::to_string(rayCount) + ", hits: " + std::to_string(hitCount);
		//logger.log(text);
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
	static long long runBDACLDefaultSchedulerBenchmark() {
		BDACL::Constructor scheduler(THREADS);
		const int TASK_COUNT = THREADS * 32;

		std::vector<std::future<void>> futures;

		for (int i = 0; i < TASK_COUNT; ++i) {
			auto promise = std::make_shared<std::promise<void>>();
			futures.push_back(promise->get_future());
			scheduler.enqueue([promise] {
				RayIntersectsTriangleSimulation_Scalable();
				promise->set_value();
				});
		}

		auto start_bdacl = std::chrono::high_resolution_clock::now();
		for (auto& fut : futures) {
			fut.get();
		}

		auto end_bdacl = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::microseconds>(end_bdacl - start_bdacl).count();
	}

	static long long runDATaskSchedulerBenchmark() {
		BDACL::DATask_Scheduler scheduler;
		const int TASK_COUNT = THREADS * 32;

		std::vector<BDACL::DATask> tasks(TASK_COUNT);
		std::vector<std::future<void>> futures;

		for (int i = 0; i < TASK_COUNT; ++i) {
			auto promise = std::make_shared<std::promise<void>>();
			futures.push_back(promise->get_future());
			tasks[i].job = [promise] {
				RayIntersectsTriangleSimulation_Scalable();
				promise->set_value();
				};
			tasks[i].jobPendingDependencies = 0;
		}

		auto start_bdacl = std::chrono::high_resolution_clock::now();
		scheduler.initSchedulerPool(THREADS);

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
	
	static long long runDATaskSchedulerBenchmarkMicro() {
		BDACL::DATask_Scheduler scheduler;
		const int TASK_COUNT = THREADS * 32;
		std::vector<BDACL::DATask> tasks(TASK_COUNT);
		std::vector<std::future<void>> futures(4);
		std::vector<std::shared_ptr<std::promise<void>>> promises(4);

		// Initialize promises
		for (int i = 0; i < 4; ++i) {
			promises[i] = std::make_shared<std::promise<void>>();
			futures[i] = promises[i]->get_future();
		}

		// Shared data - wrap in struct to avoid capture issues
		struct SharedData {
			std::vector<Triangle> triangles;
			std::unique_ptr<BVHNode> bvhRoot;
			std::vector<Ray> rays;
			std::atomic<int> hitCount{ 0 };
		} sharedData;

		BDACL::ThreadSafeLogger logger;
		scheduler.initSchedulerPool(THREADS);

		// Task 0: Generate random triangles
		{
			tasks[0].job = [&sharedData, promise = promises[0]]() {
				std::vector<Triangle> localTriangles;
				localTriangles.reserve(TRIGC);  // Reserve space first

#pragma omp parallel
				{
					std::mt19937 rng(std::random_device{}() + omp_get_thread_num());
					std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
					std::vector<Triangle> threadLocalTriangles;

#pragma omp for nowait
					for (int i = 0; i < TRIGC; ++i) {
						Vec3 v0(dist(rng), dist(rng), dist(rng));
						Vec3 v1 = v0 + Vec3(dist(rng) * 0.1f, dist(rng) * 0.1f, dist(rng) * 0.1f);
						Vec3 v2 = v0 + Vec3(dist(rng) * 0.1f, dist(rng) * 0.1f, dist(rng) * 0.1f);
						threadLocalTriangles.emplace_back(v0, v1, v2);
					}

#pragma omp critical
					localTriangles.insert(localTriangles.end(),
						threadLocalTriangles.begin(),
						threadLocalTriangles.end());
				}

				sharedData.triangles = std::move(localTriangles);
				promise->set_value();
				};
			tasks[0].jobPendingDependencies = 0;
			scheduler.addTask(&tasks[0]);
		}

		// Task 1: Build BVH (dependent on Task 0)
		{
			tasks[1].job = [&sharedData, promise = promises[1]]() {
				sharedData.bvhRoot = BVHNode::build(sharedData.triangles);
				promise->set_value();
				};
			tasks[1].jobPendingDependencies = 1;
			tasks[1].dependants.push_back(&tasks[0]);
			scheduler.addTask(&tasks[1]);
		}

		// Task 2: Generate rays (independent)
		{
			tasks[2].job = [&sharedData, promise = promises[2]]() {
				std::vector<Ray> localRays;
				localRays.reserve(RAYC);

#pragma omp parallel
				{
					std::mt19937 rng(std::random_device{}() + omp_get_thread_num());
					std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
					std::vector<Ray> threadLocalRays;

#pragma omp for nowait
					for (int i = 0; i < RAYC; ++i) {
						Vec3 origin(dist(rng), dist(rng), dist(rng));
						Vec3 dir(dist(rng), dist(rng), dist(rng));
						threadLocalRays.emplace_back(origin, dir.normalize());
					}

#pragma omp critical
					localRays.insert(localRays.end(),
						threadLocalRays.begin(),
						threadLocalRays.end());
				}

				sharedData.rays = std::move(localRays);
				promise->set_value();
				};
			tasks[2].jobPendingDependencies = 0;
			scheduler.addTask(&tasks[2]);
		}

		// Task 3: Collision test (dependent on Task 1 and 2)
		{
			tasks[3].job = [&sharedData, &logger, promise = promises[3]]() {
				int localHitCount = 0;

				if (sharedData.bvhRoot && !sharedData.rays.empty()) {
#pragma omp parallel for reduction(+:localHitCount)
					for (size_t i = 0; i < sharedData.rays.size(); ++i) {
						HitPoint hit;
						if (sharedData.bvhRoot->intersect(sharedData.rays[i], hit)) {
							localHitCount++;
						}
					}
				}

				sharedData.hitCount = localHitCount;
				logger.log("[RT] Rays: " + std::to_string(sharedData.rays.size()) +
					", hits: " + std::to_string(sharedData.hitCount.load()));
				promise->set_value();
				};
			tasks[3].jobPendingDependencies = 2;
			tasks[3].dependants.push_back(&tasks[1]);
			tasks[3].dependants.push_back(&tasks[2]);
			scheduler.addTask(&tasks[3]);
		}

		// Run tasks
		auto start = std::chrono::high_resolution_clock::now();

		// Wait for all tasks
		for (auto& future : futures) {
			future.wait();
		}

		scheduler.shutdown();
		auto end = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	}

	static long long runSIMTBenchmark() {
		std::vector<std::function<void(int)>> program = {
			[](int id) {
				RayIntersectsTriangleSimulation_Scalable();
			}
		};
		auto start_simt = std::chrono::high_resolution_clock::now();
		SIMT::Constructor simt(THREADS, program);
		simt.execute(THREADS);
		auto end_simt = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::microseconds>(end_simt - start_simt).count();
	}
#pragma endregion

}