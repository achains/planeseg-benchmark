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
        PEAC(std::string const &iniFileName): config_(iniFileName) {}
        void load_config(std::string const & config_filepath);
        int run();
     private:
        PlaneFitter pf_;
        ini_read::IniConfig config_;
    };
} // namespace peac