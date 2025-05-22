#pragma once
#include "DAGL.hpp"
#include "SIMT.hpp"
#include "BenchmarkFW.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <future>
#include <memory>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iomanip>
#include <ctime>
#include <sstream>

static constexpr int REPEAT_COUNT = 120;

// Helper function to get current timestamp as string
std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};

#ifdef _WIN32
    localtime_s(&tm_now, &time_t_now);
#else
    localtime_r(&time_t_now, &tm_now);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

// Helper function to calculate standard deviation
static double calculateStdDev(const std::vector<long long>& values, double mean) {
    double sq_sum = static_cast<double>(std::inner_product(values.begin(), values.end(), values.begin(), 0.0,
        [](double sum, double val) { return sum + val; },
        [mean](double a, double b) { return (a - mean) * (a - mean); }));

    return std::sqrt(sq_sum / static_cast<double>(values.size()));
}

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

int main(int argc, char* argv[]) {
    try {
        std::cout << "Threads: " << THREADS << std::endl;
        printArchitectureInfo();
        std::cout << "\nCopyright 2025 - Ata Ilhan Kokturk" << std::endl;
        std::cout << "WARNING: This test will do 120 repeats" << std::endl;
        std::cout << "and all of them are heavy computing jobs." << std::endl;
        std::cout << "Jobs: Raytrace "<< TRIGC <<" triangles with "<< RAYC <<" rays." << std::endl;
        std::cout << "Therefore, you should wait patiently.\n\n" << std::endl;

        // Vectors to store benchmark results
        std::vector<long long> simt_results;
        std::vector<long long> bdacs_results;

        simt_results.reserve(REPEAT_COUNT);
        bdacs_results.reserve(REPEAT_COUNT);

        // Run the benchmarks multiple times
        for (int i = 0; i < REPEAT_COUNT; ++i) {
            std::cout << "Running test " << (i + 1) << "/" << REPEAT_COUNT << std::endl;

            std::cout << "BDACS::DAGL (Depend. Aware Grouping Layer / New)" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Restabilization
            long long BDACS_Res = BenchFW::runDATaskSchedulerBenchmarkMicro(); /*BenchFW::runDATaskSchedulerBenchmarkMicro();*/ // Micro disabled due to bugs
            bdacs_results.push_back(BDACS_Res);
            std::this_thread::sleep_for(std::chrono::seconds(2)); // Restabilization
            std::cout << "SIMT (Simple Instruction, Multi-Threads / GPU-like)" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Restabilization
            long long SIMT_Res = BenchFW::runSIMTBenchmark();
            simt_results.push_back(SIMT_Res);

            // Calculate difference for this iteration
            double diff = static_cast<double>(SIMT_Res - BDACS_Res);
            double diff_pct = static_cast<double>((diff / SIMT_Res) * 100.0);

            // Print formatted results for this iteration
            std::cout << "\n--------------------------------" << std::endl;
            std::cout << "SIMT: " << SIMT_Res << " us" << std::endl;
            std::cout << "BDACS::DAGS: " << BDACS_Res << " us" << std::endl;
            std::cout << "Difference: " << diff << " us" << std::endl;
            std::cout << "Diff%: " << std::fixed << std::setprecision(2) << diff_pct << "%" << std::endl;
            std::cout << "--------------------------------\n" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        // Calculate averages
        double simt_avg = std::accumulate(simt_results.begin(), simt_results.end(), 0.0) / REPEAT_COUNT;
        double bdacs_avg = std::accumulate(bdacs_results.begin(), bdacs_results.end(), 0.0) / REPEAT_COUNT;

        // Calculate standard deviations
        double simt_stddev = calculateStdDev(simt_results, simt_avg);
        double bdacs_stddev = calculateStdDev(bdacs_results, bdacs_avg);

        // Calculate percentage standard deviations
        double simt_stddev_pct = (simt_stddev / simt_avg) * 100.0;
        double bdacs_stddev_pct = (bdacs_stddev / bdacs_avg) * 100.0;

        // Calculate difference for the averages
        double diff = simt_avg - bdacs_avg;
        double diff_pct = (diff / simt_avg) * 100.0;

        // Create CSV filename with timestamp
        std::string timestamp = getTimestamp();
        std::string someDummyText = "NONMICRO";  // User will modify this manually
        std::string filename = "result_T" + std::to_string(THREADS) + "_" + timestamp + "_" + someDummyText + ".csv";

        // Print final summary with overall average and stddev
        std::cout << "\nFINAL SUMMARY (" << REPEAT_COUNT << " tests):" << std::endl;
        std::cout << "--------------------------------" << std::endl;
        std::cout << "SIMT: " << std::fixed << std::setprecision(2) << simt_avg << " us           (stddev: " << std::fixed << std::setprecision(2) << simt_stddev_pct << "%)" << std::endl;
        std::cout << "BDACS::DAGL: " << std::fixed << std::setprecision(2) << bdacs_avg << " us        (stddev: " << std::fixed << std::setprecision(2) << bdacs_stddev_pct << "%)" << std::endl;
        std::cout << "Difference: " << std::fixed << std::setprecision(2) << diff << " us" << std::endl;
        std::cout << "Diff%: " << std::fixed << std::setprecision(2) << diff_pct << "%" << std::endl;
        std::cout << "--------------------------------" << std::endl;

        // Write results to CSV file
        std::ofstream csv_file(filename);
        if (csv_file.is_open()) {
            // Write header
            csv_file << "Threads,SIMT,BDACL,Diff%\n";

            // Write individual test results
            for (int i = 0; i < REPEAT_COUNT; ++i) {
                double diff = static_cast<double>(simt_results[i] - bdacs_results[i]);
                double diff_pct = (diff / simt_results[i]) * 100.0;

                csv_file << THREADS << ","
                    << simt_results[i] << ","
                    << bdacs_results[i] << ","
                    << std::fixed << std::setprecision(2) << diff_pct << "\n";
            }

            // Write summary line with averages (marked as 'AVG')
            csv_file << "AVG,"
                << std::fixed << std::setprecision(2) << simt_avg << ","
                << std::fixed << std::setprecision(2) << bdacs_avg << ","
                << std::fixed << std::setprecision(2) << diff_pct << "\n";

            // Write stddev line
            csv_file << "STDDEV,%,"
                << std::fixed << std::setprecision(2) << simt_stddev_pct << ","
                << std::fixed << std::setprecision(2) << bdacs_stddev_pct << "\n";

            csv_file.close();
        }
        else {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
        }

        // Print log message with filename
        std::cout << "\nResults are written to " << filename << std::endl;
        std::cout << "Enter to exit...." << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception caught at main: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "Unknown exception caught at main" << std::endl;
        return 1;
    }

    std::cin.get();
    return 0;
}