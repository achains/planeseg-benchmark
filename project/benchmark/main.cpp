#include <benchmark/benchmark.h>
#include <filesystem>

#include "peac/peac.hpp"
#include "cape/offline_runner.h"


namespace global{
    // Common globals
    constexpr const char* png_path = "/home/achains/Desktop/datasets/long_office_validation/rgbd_dataset_freiburg3_long_office_household_validation/depth";
    constexpr const char* dataset_path = "data/";
    constexpr const char* input_dir = "data/";
    // PEAC globals
    constexpr const char* peac_ini = "configs/peac-long.ini";
    // CAPE globals
    constexpr const char* cape_ini = "configs/cape-long.ini";
    constexpr const char* cape_intrinsics = "configs/cape-long-intrinsics.xml";
    constexpr const int cape_width = 640;
    constexpr const int cape_height = 480;

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

static void BM_Frame_Sequence_CAPE(benchmark::State& state) {
    // Initialize algorithm
    std::stringstream cape_ini;
    cape_ini << global::cape_ini;
    readIni(cape_ini);

    // Get intrinsics
    cv::Mat K_rgb, K_ir, dist_coeffs_rgb, dist_coeffs_ir, R_stereo, t_stereo;
    cape::loadCalibParameters(global::cape_intrinsics, K_rgb, dist_coeffs_rgb, K_ir, dist_coeffs_ir, R_stereo, t_stereo);
    float fx_ir = K_ir.at<double>(0,0); float fy_ir = K_ir.at<double>(1,1);
    float cx_ir = K_ir.at<double>(0,2); float cy_ir = K_ir.at<double>(1,2);
    float fx_rgb = K_rgb.at<double>(0,0); float fy_rgb = K_rgb.at<double>(1,1);
    float cx_rgb = K_rgb.at<double>(0,2); float cy_rgb = K_rgb.at<double>(1,2);

    int nr_horizontal_cells = global::cape_width / PATCH_SIZE;
    int nr_vertical_cells = global::cape_height / PATCH_SIZE;

    // Pre-computations for backprojection
    cv::Mat_<float> X_pre(global::cape_height,global::cape_width);
    cv::Mat_<float> Y_pre(global::cape_height,global::cape_width);
    cv::Mat_<float> U(global::cape_height,global::cape_width);
    cv::Mat_<float> V(global::cape_height,global::cape_width);
    for (int r=0;r<global::cape_height; r++){
        for (int c=0;c<global::cape_width; c++){
            // Not efficient but at this stage doesn't t matter
            X_pre.at<float>(r,c) = (c-cx_ir)/fx_ir; Y_pre.at<float>(r,c) = (r-cy_ir)/fy_ir;
        }
    }

    // Pre-computations for maping an image point cloud to a cache-friendly array where cell's local point clouds are contiguous
    cv::Mat_<int> cell_map(global::cape_height,global::cape_width);

    for (int r=0;r<global::cape_height; r++){
        int cell_r = r/PATCH_SIZE;
        int local_r = r%PATCH_SIZE;
        for (int c=0;c<global::cape_width; c++){
            int cell_c = c/PATCH_SIZE;
            int local_c = c%PATCH_SIZE;
            cell_map.at<int>(r,c) = (cell_r*nr_horizontal_cells+cell_c)*PATCH_SIZE*PATCH_SIZE + local_r*PATCH_SIZE + local_c;
        }
    }

    cv::Mat_<float> X(global::cape_height,global::cape_width);
    cv::Mat_<float> Y(global::cape_height,global::cape_width);
    Eigen::MatrixXf cloud_array(global::cape_width*global::cape_height,3);
    Eigen::MatrixXf cloud_array_organized(global::cape_width*global::cape_height,3);

    auto algorithm = CAPE(global::cape_height, global::cape_width, PATCH_SIZE, PATCH_SIZE, false, COS_ANGLE_MAX, MAX_MERGE_DIST);

    // List png frames
    std::filesystem::path data_dir(global::png_path);
    std::filesystem::directory_iterator dir_it(data_dir);

    std::vector<std::filesystem::path> data_entries(std::filesystem::begin(dir_it), std::filesystem::end(dir_it));
    std::sort(data_entries.begin(), data_entries.end());
    // Benchmark
    for (auto _ : state){
        for (size_t i = 0; i < std::min(state.range(0), static_cast<int64_t>(data_entries.size())); ++i){
            // Setting up
            state.PauseTiming();
            auto d_img = cv::imread(data_entries[i], cv::IMREAD_ANYDEPTH);
            d_img.convertTo(d_img, CV_32F);

            // Backproject to point cloud
            X = X_pre.mul(d_img); Y = Y_pre.mul(d_img);
            cloud_array.setZero();

            cape::projectPointCloud(X, Y, d_img, U, V, fx_rgb, fy_rgb, cx_rgb, cy_rgb, t_stereo.at<double>(2), cloud_array);

            cv::Mat_<cv::Vec3b> seg_rz = cv::Mat_<cv::Vec3b>(global::cape_height,global::cape_width,cv::Vec3b(0,0,0));
            cv::Mat_<uchar> seg_output = cv::Mat_<uchar>(global::cape_height,global::cape_width,uchar(0));

            int nr_planes, nr_cylinders;
            std::vector<PlaneSeg> plane_params;
            std::vector<CylinderSeg> cylinder_params;
            state.ResumeTiming();
            // Running algorithm
            cape::organizePointCloudByCell(cloud_array, cloud_array_organized, cell_map);
            algorithm.process(cloud_array_organized, nr_planes, nr_cylinders, seg_output, plane_params, cylinder_params);
        }
    }
}

// Arg -- Number of frames to take
BENCHMARK(BM_Frame_Sequence_PEAC)->Arg(1000)->Unit(benchmark::TimeUnit::kSecond);
BENCHMARK(BM_Frame_Sequence_CAPE)->Arg(1000)->Unit(benchmark::TimeUnit::kSecond);


BENCHMARK_MAIN();
