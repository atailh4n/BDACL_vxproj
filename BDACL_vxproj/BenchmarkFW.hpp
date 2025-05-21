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
static constexpr int RAYC = 500;
static constexpr int TRIGC = 2'000;

using namespace KGL;

namespace BenchFW {
	static void RayIntersectsTriangleSimulation_Scalable_MicroRevised(int triangleCount = TRIGC, int rayCount = RAYC) {
		// Define the logger and daTaskSch.
		BDACL::ThreadSafeLogger logger;
		BDACL::DATask_Scheduler daTaskSch;
		// Wait for tasks finish. We use future.
		std::promise<void> promise;
		std::future<void> future = promise.get_future();

		// Initialize pool.
		daTaskSch.initSchedulerPool(THREADS);
		// Do jobs here.
		BDACL::DATask task1, task2, task3, task4;
		// Pass address to lambda
		{
			task1.job = [&logger] {
				logger.log("Hello World!"); // Logger is needed because to prevent race cond.
				};
		}
		{
			task2.job = [&logger, &promise] {
				logger.log("This one must wait for task1!");
				promise.set_value(); // Last job is finished.
			};
		}

		daTaskSch.addDependant(&task1, &task2); // Task1 = Parent. Task2 = Child.
		daTaskSch.addTask(&task1); // Just run task1, the start point.
		// Only add undependent and syncronous tasks. If we had any task
		// like task1, we should started it like the example.

		// Wait for the finish.
		future.get();

		// After finish, notify the scheduler.
		daTaskSch.shutdown();
		
		// Triangle produce
		std::vector<Triangle> triangles;
		triangles.reserve(triangleCount);

		std::mt19937 rng(std::random_device{}());
		std::uniform_real_distribution<float> dist(-100.0f, 100.0f);


		//gen random trigs
		#pragma omp parallel for
		for (size_t i = 0; i < triangleCount; ++i)
		{
			Vec3 v0(dist(rng), dist(rng), dist(rng));
			Vec3 v1 = v0 + Vec3(dist(rng) * 0.1f, dist(rng) * 0.1f, dist(rng) * 0.1f);
			Vec3 v2 = v0 + Vec3(dist(rng) * 0.1f, dist(rng) * 0.1f, dist(rng) * 0.1f);
			triangles.emplace_back(v0, v1, v2);
		}

		//BVH node create
		auto bvhRoot = BVHNode::build(triangles);

		//rt
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

	}
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
	//static long long runBDACLDefaultSchedulerBenchmark() {
	//	BDACL::Constructor scheduler(THREADS);
	//	const int TASK_COUNT = THREADS * 32;

	//	std::vector<std::future<void>> futures;

	//	for (int i = 0; i < TASK_COUNT; ++i) {
	//		auto promise = std::make_shared<std::promise<void>>();
	//		futures.push_back(promise->get_future());
	//		scheduler.enqueue([promise] {
	//			RayIntersectsTriangleSimulation_Scalable();
	//			promise->set_value();
	//			});
	//	}

	//	auto start_bdacl = std::chrono::high_resolution_clock::now();
	//	for (auto& fut : futures) {
	//		fut.get();
	//	}

	//	auto end_bdacl = std::chrono::high_resolution_clock::now();
	//	return std::chrono::duration_cast<std::chrono::microseconds>(end_bdacl - start_bdacl).count();
	//}

	//static long long runDATaskSchedulerBenchmark() {
	//	BDACL::DATask_Scheduler scheduler;
	//	const int TASK_COUNT = THREADS * 32;

	//	std::vector<BDACL::DATask> tasks(TASK_COUNT);
	//	std::vector<std::future<void>> futures;

	//	for (int i = 0; i < TASK_COUNT; ++i) {
	//		auto promise = std::make_shared<std::promise<void>>();
	//		futures.push_back(promise->get_future());
	//		tasks[i].job = [promise] {
	//			RayIntersectsTriangleSimulation_Scalable();
	//			promise->set_value();
	//			};
	//		tasks[i].jobPendingDependencies = 0;
	//	}

	//	auto start_bdacl = std::chrono::high_resolution_clock::now();
	//	scheduler.initSchedulerPool(THREADS);

	//	// Tüm task'ları sırayla ekle
	//	for (auto& task : tasks) {
	//		scheduler.addTask(&task);
	//	}

	//	for (auto& fut : futures) {
	//		fut.get();
	//	}

	//	auto end_bdacl = std::chrono::high_resolution_clock::now();
	//	return std::chrono::duration_cast<std::chrono::microseconds>(end_bdacl - start_bdacl).count();
	//}
	
	// FIXME: 2GB RAM ALLOC?????!
   // static long long runDATaskSchedulerBenchmarkMicro() {
   //     BDACL::DATask_Scheduler scheduler;

   //     constexpr int TASK_COUNT = THREADS * 32;
   //     std::vector<BDACL::DATask> tasks(TASK_COUNT * 4);

   //     auto promise = std::make_shared<std::promise<void>>();
   //     std::future<void> future = promise->get_future();

   //     BDACL::ThreadSafeLogger logger;

   //     // Shared data - wrap in struct to avoid capture issues
   //     struct SharedData {
   //         std::vector<std::vector<Triangle>> triangles;
   //         std::vector<std::unique_ptr<BVHNode>> bvhRoots;
   //         std::vector<std::vector<Ray>> rays;
   //         std::vector<int> hitCounts; // Changed from atomic to regular int
   //         std::atomic<int> completedTasks{ 0 }; // Track completed tasks
   //     };

   //     std::shared_ptr<SharedData> sharedData = std::make_shared<SharedData>();
   //     sharedData->triangles.resize(TASK_COUNT);
   //     sharedData->bvhRoots.resize(TASK_COUNT);
   //     sharedData->rays.resize(TASK_COUNT);
   //     sharedData->hitCounts.resize(TASK_COUNT, 0); // Initialize all hit counts to 0

   //     // Mutex for protecting hitCounts access
   //     std::shared_ptr<std::mutex> hitCountMutex = std::make_shared<std::mutex>();

   //     scheduler.initSchedulerPool(THREADS);

   //     // Create TASK_COUNT sets of tasks
   //     for (int i = 0; i < TASK_COUNT; i++) {
   //         // Task 0: Generate random triangles
   //         {
   //             tasks[i * 4 + 0].job = [sharedData, i, &logger]() {
   //                 {
			//			std::vector<Triangle> triangles(TRIGC);
			//			#pragma omp parallel
			//			{
			//				std::mt19937 rng(std::random_device{}() + omp_get_thread_num());
			//				std::uniform_real_distribution<float> dist(-100.0f, 100.0f);

			//				#pragma omp for
			//				for (int j = 0; j < TRIGC; ++j) {
			//					Vec3 v0(dist(rng), dist(rng), dist(rng));
			//					Vec3 v1 = v0 + Vec3(dist(rng) * 0.1f, dist(rng) * 0.1f, dist(rng) * 0.1f);
			//					Vec3 v2 = v0 + Vec3(dist(rng) * 0.1f, dist(rng) * 0.1f, dist(rng) * 0.1f);
			//					triangles[j] = Triangle(v0, v1, v2);
			//				}
			//			}

			//			sharedData->triangles[i] = std::move(triangles);
   //                 }
   //                 };
   //             tasks[i * 4 + 0].jobPendingDependencies = 0;
   //         }

   //         // Task 1: Build BVH (dependent on Task 0)
   //         {
   //             tasks[i * 4 + 1].job = [sharedData, i, &logger]() {
   //                 sharedData->bvhRoots[i] = BVHNode::build(sharedData->triangles[i]);
   //                 };
   //             tasks[i * 4 + 1].jobPendingDependencies = 1;
   //         }

   //         // Task 2: Generate rays (independent)
   //         {
   //             tasks[i * 4 + 2].job = [sharedData, i, &logger]() {
   //                 std::vector<Ray> localRays;
   //                 localRays.reserve(RAYC);

   //                 {
   //                     std::mt19937 rng(std::random_device{}() + omp_get_thread_num());
   //                     std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
   //                     std::vector<Ray> threadLocalRays;
			//			#pragma omp parallel for
   //                     for (int j = 0; j < RAYC; ++j) {
   //                         Vec3 origin(dist(rng), dist(rng), dist(rng));
   //                         Vec3 dir(dist(rng), dist(rng), dist(rng));
   //                         threadLocalRays.emplace_back(origin, dir.normalize());
   //                     }

   //                     localRays.insert(localRays.end(),
   //                         threadLocalRays.begin(),
   //                         threadLocalRays.end());
   //                 }

   //                 sharedData->rays[i] = std::move(localRays);
   //                 };
   //             tasks[i * 4 + 2].jobPendingDependencies = 0;
   //         }

   //         // Task 3: Collision test (dependent on Task 1 and 2)
   //         {
   //             tasks[i * 4 + 3].job = [sharedData, &logger, promise, i, hitCountMutex]() {
   //                 int localHitCount = 0;
   //                 if (sharedData->bvhRoots[i] && !sharedData->rays[i].empty()) {
			//			#pragma omp parallel for
   //                     for (size_t j = 0; j < sharedData->rays[i].size(); ++j) {
   //                         HitPoint hit;
   //                         if (sharedData->bvhRoots[i]->intersect(sharedData->rays[i][j], hit)) {
   //                             localHitCount++;
   //                         }
   //                     }
   //                 }

   //                 // Safely update hit count using mutex
   //                 {
   //                     std::lock_guard<std::mutex> lock(*hitCountMutex);
   //                     sharedData->hitCounts[i] = localHitCount;
   //                 }

   //                 // Increase completed task count
   //                 int completedTasks = ++sharedData->completedTasks;

   //                 // Only set the promise value when ALL tasks are completed
   //                 if (completedTasks == TASK_COUNT) {
   //                     promise->set_value();
   //                 }
   //             };
   //             tasks[i * 4 + 3].jobPendingDependencies = 2;
   //         }

   //         // Set dependencies for this iteration
			//scheduler.addDependant(&tasks[i * 4 + 0], &tasks[i * 4 + 1]);
			//scheduler.addDependant(&tasks[i * 4 + 1], &tasks[i * 4 + 3]);
			//scheduler.addDependant(&tasks[i * 4 + 2], &tasks[i * 4 + 3]);

   //         // Add tasks to scheduler
   //         scheduler.addTask(&tasks[i * 4 + 0]);
   //         scheduler.addTask(&tasks[i * 4 + 1]);
   //         scheduler.addTask(&tasks[i * 4 + 2]);
   //         scheduler.addTask(&tasks[i * 4 + 3]);
   //     }

   //     // Run tasks
   //     auto start = std::chrono::high_resolution_clock::now();

   //     // Wait for all tasks
   //     future.wait();

   //     scheduler.shutdown();
   //     auto end = std::chrono::high_resolution_clock::now();

   //     return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
   // }

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