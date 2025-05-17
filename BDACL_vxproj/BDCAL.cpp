#include "BDCAL.hpp"

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
