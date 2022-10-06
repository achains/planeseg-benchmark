#include <benchmark/benchmark.h>
#include <filesystem>

#include "peac/peac.hpp"


namespace global{
    constexpr const char* dataset_path = "data/";
    constexpr const char* input_dir = "data/";
    constexpr const char* peac_ini = "configs/peac-long.ini";
} // namespace global

static void BM_Frame_Sequence_PEAC(benchmark::State& state) {
    // Initialize algorithm
    auto algorithm = peac::PEAC(global::peac_ini);
    // List pcd frames
    std::filesystem::path data_dir(global::input_dir);
    std::filesystem::directory_iterator dir_it(data_dir);

    std::vector<std::filesystem::path> data_entries(std::filesystem::begin(dir_it), std::filesystem::end(dir_it));
    std::sort(data_entries.begin(), data_entries.end());
    // Benchmark
    for (auto _ : state){
        for (size_t i = 0; i < std::min(state.range(0), static_cast<int64_t>(data_entries.size())); ++i){
            // Reading PCD
            state.PauseTiming();
            auto pcd = algorithm.read_pcd(data_entries[i]);
            std::vector<std::vector<int>> labels;
            state.ResumeTiming();
            // Running algorithm
            algorithm.run(&pcd, &labels);
        }
    }
}

// Arg -- Number of frames to take
BENCHMARK(BM_Frame_Sequence_PEAC)->Arg(1);

BENCHMARK_MAIN();
