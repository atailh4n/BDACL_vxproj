#include "SIMT.hpp"
#include <omp.h>

void SIMT::Constructor::execute(int threadCount)
{
	if (threadCount <= 0) {
		threadCount = 1;  // Default 1 thread
	}

	std::vector<std::thread> threads;
	threads.reserve(threadCount);

	for (int t = 0; t < threadCount; ++t) {
		threads.emplace_back([this, t, threadCount]() {
			for (int w = t; w < warpCount && w < (int)warps.size(); w += threadCount) {
				for (size_t instr = 0; instr < warps[w].size(); ++instr) {
					for (int lane = 0; lane < WARP_SIZE; ++lane) {
						if (warps[w][instr]) {
							warps[w][instr](w* WARP_SIZE + lane);
						}
					}
				}
			}
		});
	}

	#pragma omp parallel for
	for (auto& th : threads) {
		th.join();
	}
}
