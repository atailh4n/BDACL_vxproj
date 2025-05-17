#pragma once
#include "DAGL.hpp"
#include "SIMT.hpp"
#include "BenchmarkFW.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <future>
#include <memory>

static void printArchitectureInfo() {
    std::cout << "Simulated Architecture: " << std::endl;
#if defined(__x86_64__) || defined(_M_X64)
    std::cout << "CPU: x86_64\n";
#elif defined(__aarch64__)
    std::cout << "CPU: ARM64\n";
#else
    std::cout << "CPU: Unknown\n";
#endif
    std::cout << "Execution Environment: CPU-based GPU simulation\n";
}

// Test kodu
//void runParallelBenchmark() {
//    BDACL::DATask_Scheduler scheduler;
//    const int TASK_COUNT = 32;
//    const int THREAD_COUNT = 32;
//
//    std::vector<BDACL::DATask> tasks(TASK_COUNT);
//    std::vector<std::future<void>> futures;
//
//    auto start = std::chrono::high_resolution_clock::now();
//
//    // Tüm task'ları hazırla
//    for (int i = 0; i < TASK_COUNT; ++i) {
//        auto promise = std::make_shared<std::promise<void>>();
//        futures.push_back(promise->get_future());
//
//        tasks[i].job = [promise] {
//            try {
//                BenchFW::RayIntersectsTriangleSimulation_Scalable(10000, 1000);
//                promise->set_value();
//            }
//            catch (...) {
//                try {
//                    promise->set_exception(std::current_exception());
//                }
//                catch (...) {
//                    // promise zaten set edilmiş olabilir, sessiz geç
//                }
//            }
//            };
//
//        tasks[i].jobPendingDependencies = 0;
//    }
//
//    // 32 threadle çalıştır
//    scheduler.runAwaitingTasks(THREAD_COUNT);
//
//    // Task'ları ekle
//    for (auto& task : tasks) {
//        scheduler.addTask(&task);
//    }
//
//    // Tüm task'ların bitmesini bekle
//    for (auto& fut : futures) {
//        fut.get();
//    }
//
//    auto end = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//
//    std::cout << "32 Thread DATask_Scheduler Elapsed time: " << duration.count() << " us\n";
//}

int main(int argc, char* argv[]) {
    try {
        int threads = 1; // default
        bool threadsFlagUsed = false;

        // Komut satırı argümanlarını işle
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if (arg == "--threads") {
                if (threadsFlagUsed) {
                    std::cerr << "Hata: --threads birden fazla kez kullanılamaz!" << std::endl;
                    return 1;
                }
                if (i + 1 >= argc) {
                    std::cerr << "Hata: --threads sonrası bir sayı bekleniyor!" << std::endl;
                    return 1;
                }

                try {
                    threads = std::stoi(argv[i + 1]);
                    if (threads < 1) {
                        std::cerr << "Hata: threads pozitif bir sayı olmalı!" << std::endl;
                        return 1;
                    }
                    threadsFlagUsed = true;
                    ++i; // sayıyı atla
                }
                catch (...) {
                    std::cerr << "Hata: Geçersiz sayı formatı: " << argv[i + 1] << std::endl;
                    return 1;
                }
            }
        }

        std::cout << "Threads: " << threads << std::endl;
        printArchitectureInfo();

        std::cout << "\nCopyright 2025 - Ata Ilhan Kokturk" << std::endl;
        std::cout << "WARNING: This test will do 120 repeats" << std::endl;
        std::cout << "and all of them are heavy computing jobs." << std::endl;
        std::cout << "Jobs: Raytrace 20K triangles with 4K rays." << std::endl;
        std::cout << "Therefore, you should wait patiently.\n\n" << std::endl;

        constexpr int REPEAT_COUNT = 120;
        std::cout << "DAGL" << std::endl;
        long long BCADL_Res = BenchFW::runDATaskSchedulerBenchmark(threads * 32); // close the difference between DATask and WARP
        std::cout << "SIMT" << std::endl;
        long long SIMT_Res = BenchFW::runSIMTBenchmark(threads);

        std::cout << "BDACL: " << std::to_string(BCADL_Res) << std::endl;
        std::cout << "SIMT: " << std::to_string(SIMT_Res) << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
        return 1;
    }
    std::cin.get();
    return 0;
}