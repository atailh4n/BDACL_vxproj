#pragma once
#include <vector>
#include <functional>
#include <thread>

namespace SIMT {
	class Constructor {
		static constexpr int WARP_SIZE = 32; // NVIDIA = 32, AMD = 64
		static constexpr int UNIT_COUNT = 1;
		int warpCount;
		std::vector<std::vector<std::function<void(int)>>> warps;


	public:
		Constructor(int count, const std::vector<std::function<void(int)>>& prog)
			: warpCount(count) {
			warps.resize(warpCount, prog);
		};

		void execute(int threadCount);
	};
}