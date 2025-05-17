#include "SIMT.hpp"

void SIMT::Constructor::execute(int threadCount = -1)
{
	if (threadCount <= 0) {
		threadCount = UNIT_COUNT;  // default deðer olarak UNIT_SIZE kullanýlýr
	}
	std::vector<std::thread> threads;

	for (int t = 0; t < UNIT_COUNT; ++t) {
		threads.emplace_back([this, t, threadCount]() {
			for (size_t instr = 0; instr < warps[0].size(); ++instr) {
				for (int w = t; w < warpCount; w += threadCount) {
					for (int lane = 0; lane < WARP_SIZE; ++lane) {
						if (warps[w][instr]) {
							warps[w][instr](w* WARP_SIZE + lane);
						}
					}
				}
			}
			});
	}

	for (auto& th : threads) {
		th.join();
	}
}
