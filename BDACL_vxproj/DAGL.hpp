#pragma once
#include <iostream>
#include <vector>
#include <functional>
#include <mutex>
#include <atomic>
#include <deque>
#include <thread>
#include <condition_variable>
#include <memory>
#include <stdexcept>

namespace BDACL {
	struct DATask {
		std::vector<DATask*> dependants;
		std::function<void()> job;
		std::atomic<int> jobPendingDependencies;
	};

	class ThreadSafeLogger {
		std::mutex cout_mutex;
	public:
		void log(const std::string& message) {
			std::lock_guard<std::mutex> lock(cout_mutex);
			std::cout << message << std::endl;
		}
	};

	class DATask_Scheduler {
		struct ThreadLocalQueue {
			std::deque<DATask*> tasks;
			std::mutex mutex;
			std::condition_variable cv;
			bool notified = false;
		};

		std::vector<std::unique_ptr<ThreadLocalQueue>> threadQueues;
		std::vector<std::thread> threads;
		std::atomic<bool> stop{ false };
		std::atomic<size_t> currentQueueIndex{ 0 };
		std::mutex globalMutex;

		// Work-stealing için optimize edilmiþ fonksiyon
		bool tryStealWork(DATask*& task, size_t thiefIndex) {
			const size_t numQueues = threadQueues.size();
			for (size_t i = 1; i < numQueues; ++i) {
				const size_t victimIndex = (thiefIndex + i) % numQueues;
				auto& victimQueue = *threadQueues[victimIndex];

				std::unique_lock<std::mutex> lock(victimQueue.mutex, std::try_to_lock);
				if (lock && !victimQueue.tasks.empty()) {
					task = victimQueue.tasks.back();
					victimQueue.tasks.pop_back();
					return true;
				}
			}
			return false;
		}

	public:
		~DATask_Scheduler() {
			shutdown();
		}

		void addTask(DATask* task) {
			if (!task || stop.load()) return;
			size_t index = currentQueueIndex.fetch_add(1, std::memory_order_relaxed) % threadQueues.size();
			auto& queue = *threadQueues[index];

			{
				std::lock_guard<std::mutex> lock(queue.mutex);
				queue.tasks.push_front(task);
				queue.notified = true;
			}
			queue.cv.notify_one();
		}

		void runAwaitingTasks(int threadCount = -1) {
			if (threadCount <= 0) threadCount = std::thread::hardware_concurrency();
			if (!threads.empty()) return; // Zaten çalýþýyor

			// Önce tüm kuyruklarý oluþtur
			threadQueues.resize(threadCount);
			for (int i = 0; i < threadCount; ++i) {
				threadQueues[i] = std::make_unique<ThreadLocalQueue>();
			}

			// Sonra thread'leri baþlat
			threads.reserve(threadCount);
			for (int i = 0; i < threadCount; ++i) {
				threads.emplace_back([this, i] {
					auto& localQueue = *threadQueues[i];

					while (!stop.load()) {
						DATask* task = nullptr;

						// 1. Önce yerel kuyruktan al
						{
							std::unique_lock<std::mutex> lock(localQueue.mutex);
							if (!localQueue.tasks.empty()) {
								task = localQueue.tasks.front();
								localQueue.tasks.pop_front();
							}
						}

						// 2. Yerel kuyruk boþsa work-stealing yap
						if (!task) {
							task = tryGetTaskFromOtherQueue(i);
						}

						// 3. Hala task yoksa bekle
						if (!task) {
							std::unique_lock<std::mutex> lock(localQueue.mutex);
							localQueue.cv.wait(lock, [this, &localQueue] {
								return stop.load() || localQueue.notified;
								});
							localQueue.notified = false;
							continue;
						}
						// Task'ý çalýþtýr
						executeTaskSafely(task);
					}
				});
			}
		}

	private:
		DATask* tryGetTaskFromOtherQueue(size_t myIndex) {
			DATask* task = nullptr;
			if (tryStealWork(task, myIndex)) {
				return task;
			}
			return nullptr;
		}

		void executeTaskSafely(DATask* task) {
			try {
				if (task) {
					task->job();
					markTaskFinished(task);
				}
			}
			catch (...) {
				// Hatalarý logla ama uygulamayý durdurma
				std::lock_guard<std::mutex> lock(globalMutex);
				std::cerr << "Task execution failed" << std::endl;
			}
		}

	public:
		void markTaskFinished(DATask* task) {
			if (!task) return;

			for (auto dep : task->dependants) {
				if (!dep) continue;

				// Atomik olarak dependency sayýsýný azalt
				int remaining = --dep->jobPendingDependencies;
				if (remaining == 0) {
					addTask(dep);
				}
			}
		}

		void shutdown() {
			stop.store(true);

			// Tüm thread'leri uyandýr
			for (auto& queue : threadQueues) {
				if (queue) {
					std::lock_guard<std::mutex> lock(queue->mutex);
					queue->notified = true;
					queue->cv.notify_all();
				}
			}

			// Thread'leri bekle
			for (auto& thread : threads) {
				if (thread.joinable()) {
					thread.join();
				}
			}

			threads.clear();
			threadQueues.clear();
		}
	};
}