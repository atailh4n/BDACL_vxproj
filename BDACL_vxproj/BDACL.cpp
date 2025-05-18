#include "BDACL.hpp"

void BDACL::Constructor::runInternalProc(int id)
{
	while (running) {
		std::function<void()> task;
		{
			std::unique_lock<std::mutex> lock(mtx);
			cv.wait(lock, [&]() {
				return !unitQueues[id].empty() || !running;
				});
			if (!running) break;
			if (!unitQueues[id].empty()) {
				task = std::move(unitQueues[id].front());
				unitQueues[id].pop();
			}
		}
		if (task) task();
	}
}

void BDACL::Constructor::enqueue(const std::function<void()>& task)
{
	{
		std::lock_guard<std::mutex> lock(mtx);
		// Find the unit with the smallest queue
		size_t minQueueIdx = 0;
		size_t minQueueSize = unitQueues[0].size();
		#pragma omp parallel for
		for (size_t i = 0; i < unitQueues.size(); ++i) {
			if (unitQueues[i].size() < minQueueSize) {
				#pragma omp critical
				{
					minQueueSize = unitQueues[i].size();
					minQueueIdx = i;
				}
			}
		}

		// Add task to the unit with the smallest queue
		unitQueues[minQueueIdx].push(task);
	}
	cv.notify_all();
}
