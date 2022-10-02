//
// Created by achains on 25.09.22.
//
#pragma once

#include <string>
#include "peac/peac_utils.hpp"

namespace peac {
    using namespace peac_utils;

    class PEAC{
     public:
        PEAC(std::string const &iniFileName): config_(iniFileName) {
            init_pf_params();
        }
        int run(const ImageXYZ* pointsIn, std::vector<std::vector<int>>* labelsOut);

        ImageXYZ read_pcd(std::string const &pcdFileName);
     private:
        void init_pf_params();
        PlaneFitter pf_;
        ini_read::IniConfig config_;
    };
} // namespace peac