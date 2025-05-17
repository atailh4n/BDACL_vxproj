#pragma once
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <functional>

namespace BDACL {
	class Constructor {
		static const int UNIT_COUNT = 1; // Simulates dual RTX-like cores
		std::vector<std::thread> units;
		std::vector<std::queue<std::function<void()>>> unitQueues;
		std::atomic<bool> running{ true };
		std::mutex mtx;
		std::condition_variable cv;

		void runInternalProc(int id);

	public:
		Constructor(int threadCount = -1) {
			if (threadCount <= 0) threadCount = UNIT_COUNT;
			unitQueues.resize(threadCount);
			for (int i = 0; i < threadCount; ++i) {
				units.emplace_back([this, i]() { runInternalProc(i); });
			}
		}

		~Constructor() {
			running = false;
			cv.notify_all();
			for (auto& t : units) if (t.joinable()) t.join();
		}

		void enqueue(const std::function<void()>& task);
	};
}