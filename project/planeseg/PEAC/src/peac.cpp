#include "peac/peac_utils.hpp"
#include "peac/peac.hpp"

using namespace peac_utils;

namespace peac {
    void PEAC::init_pf_params() {
        // Setup fitter
        pf_.minSupport = config_.iniGet<int>("minSupport", 3000);
        pf_.windowWidth = config_.iniGet<int>("windowWidth", 10);
        pf_.windowHeight = config_.iniGet<int>("windowHeight", 10);
        pf_.doRefine = config_.iniGet<int>("doRefine", 1) != 0;

        pf_.params.initType = static_cast<ahc::InitType>(
                config_.iniGet("initType", static_cast<int>(pf_.params.initType)));

        // T_mse
        pf_.params.stdTol_merge = config_.iniGet("stdTol_merge", pf_.params.stdTol_merge);
        pf_.params.stdTol_init = config_.iniGet("stdTol_init", pf_.params.stdTol_init);
        pf_.params.depthSigma = config_.iniGet("depthSigma", pf_.params.depthSigma);

        //T_dz
        pf_.params.depthAlpha = config_.iniGet("depthAlpha", pf_.params.depthAlpha);
        pf_.params.depthChangeTol = config_.iniGet("depthChangeTol", pf_.params.depthChangeTol);

        //T_ang
        pf_.params.z_near = config_.iniGet("z_near", pf_.params.z_near);
        pf_.params.z_far = config_.iniGet("z_far", pf_.params.z_far);
        pf_.params.angle_near = MACRO_DEG2RAD(config_.iniGet("angleDegree_near",MACRO_RAD2DEG(pf_.params.angle_near)));
        pf_.params.angle_far = MACRO_DEG2RAD(config_.iniGet("angleDegree_far",MACRO_RAD2DEG(pf_.params.angle_far)));
        pf_.params.similarityTh_merge = std::cos(MACRO_DEG2RAD(config_.iniGet("similarityDegreeTh_merge",MACRO_RAD2DEG(pf_.params.similarityTh_merge))));
        pf_.params.similarityTh_refine = std::cos(MACRO_DEG2RAD(config_.iniGet("similarityDegreeTh_refine",MACRO_RAD2DEG(pf_.params.similarityTh_refine))));
    }

    // TODO: Memory leak problems (!)
    ImageXYZ PEAC::read_pcd(std::string const &pcdFileName) {
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFileName, *cloud)) {
            PCL_ERROR ("ERROR: Could not read input point cloud %s.\n", pcdFileName.c_str());
        } else {
            pcl::transformPointCloud<pcl::PointXYZ>(
                    *cloud, *cloud,
                    Eigen::Affine3f(Eigen::UniformScaling<float>(
                            static_cast<float>(config_.iniGet<double>("unitScaleFactor", 1.0f)))));
        }
        std::cout << "OK\n";
        return ImageXYZ(cloud);
    }

    int PEAC::run(const ImageXYZ* pointsIn, std::vector<std::vector<int>>* labelsOut){
        pf_.run(pointsIn, labelsOut, 0, 0, false);
        std::cout << "Number of extracted planes: " << labelsOut->size() << '\n';
        return 0;
    }
} // namespace peac