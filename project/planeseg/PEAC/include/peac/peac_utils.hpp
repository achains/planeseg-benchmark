//
// Created by achains on 25.09.22.
//
#pragma once
#pragma warning(disable: 4996)
#pragma warning(disable: 4819)
#define _CRT_SECURE_NO_WARNINGS

#include <map>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/common/transforms.h>

#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>

#include "AHCPlaneFitter.hpp"

namespace peac_utils {
    template<class PointT>
    struct OrganizedImage3D {
        std::shared_ptr<pcl::PointCloud<PointT>> cloud;
        //note: ahc::PlaneFitter assumes mm as unit!!!
        const double unitScaleFactor;

        OrganizedImage3D(const pcl::PointCloud<PointT>& c) : cloud(c), unitScaleFactor(1) {}
        OrganizedImage3D(const OrganizedImage3D& other) : cloud(other.cloud), unitScaleFactor(other.unitScaleFactor) {}
        OrganizedImage3D(std::shared_ptr<pcl::PointCloud<PointT>> p_cloud) : cloud(p_cloud), unitScaleFactor(1) {}

        inline int width() const { return cloud->width; }
        inline int height() const { return cloud->height; }
        inline bool get(const int row, const int col, double& x, double& y, double& z) const {
            const PointT& pt=cloud->at(col,row);
            x=pt.x*unitScaleFactor; y=pt.y*unitScaleFactor; z=pt.z*unitScaleFactor; //TODO: will this slowdown the speed?

            return std::isnan(z)==0; //return false if current depth is NaN
        }
    };
    typedef OrganizedImage3D<pcl::PointXYZ> ImageXYZ;
    typedef ahc::PlaneFitter< ImageXYZ > PlaneFitter;
    typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

    namespace ini_read {
#ifdef _WIN32
        const char filesep = '\\';
#else
        const char filesep = '/';
#endif
        // similar to matlab's fileparts
        // if in=parent/child/file.txt
        // then path=parent/child
        // name=file, ext=txt
        void fileparts(const std::string& str, std::string* pPath=0,
                       std::string* pName=0, std::string* pExt=0)
        {
            std::string::size_type last_sep = str.find_last_of(filesep);
            std::string::size_type last_dot = str.find_last_of('.');
            if (last_dot<last_sep) // "D:\parent\child.folderA\file", "D:\parent\child.folderA\"
                last_dot = std::string::npos;

            std::string path, name, ext;

            if (last_sep==std::string::npos) {
                path = ".";
                if(last_dot==std::string::npos) { // "test"
                    name = str;
                    ext = "";
                } else { // "test.txt"
                    name = str.substr(0, last_dot);
                    ext = str.substr(last_dot+1);
                }
            } else {
                path = str.substr(0, last_sep);
                if(last_dot==std::string::npos) { // "d:/parent/test", "d:/parent/child/"
                    name = str.substr(last_sep+1);
                    ext = "";
                } else { // "d:/parent/test.txt"
                    name = str.substr(last_sep+1, last_dot-last_sep-1);
                    ext = str.substr(last_dot+1);
                }
            }

            if(pPath!=0) {
                *pPath = path;
            }
            if(pName!=0) {
                *pName = name;
            }
            if(pExt!=0) {
                *pExt = ext;
            }
        }
        //"D:/test/test.txt" -> "D:/test/"
        std::string getFileDir(const std::string &fileName)
        {
            std::string path;
            fileparts(fileName, &path);
            return path;
        }
        //"D:/parent/test.txt" -> "test"
        //"D:/parent/test" -> "test"
        std::string getNameNoExtension(const std::string &fileName)
        {
            std::string name;
            fileparts(fileName, 0, &name);
            return name;
        }
        struct IniConfig{
            IniConfig(std::string const &iniFileName){
                iniLoad(iniFileName);
            }

            std::map<std::string, std::string> ini;

            void iniLoad(std::string iniFileName) {
                std::ifstream in(iniFileName);
                if(!in.is_open()) {
                    std::cout<<"[iniLoad] "<<iniFileName<<" not found, use default parameters!"<<std::endl;
                    return;
                }
                while(in) {
                    std::string line;
                    std::getline(in, line);
                    if(line.empty() || line[0]=='#') continue;
                    std::string key, value;
                    size_t eqPos = line.find_first_of("=");
                    if(eqPos == std::string::npos || eqPos == 0) {
                        // std::cout<<"[iniLoad] ignore line:"<<line<<std::endl;
                        continue;
                    }
                    key = line.substr(0,eqPos);
                    value = line.substr(eqPos+1);
                    // std::cout<<"[iniLoad] "<<key<<"=>"<<value<<std::endl;
                    ini[key]=value;
                }
            }
            template<class T>
            T iniGet(std::string key, T default_value) {
                std::map<std::string, std::string>::const_iterator itr=ini.find(key);
                if(itr!=ini.end()) {
                    std::stringstream ss;
                    ss<<itr->second;
                    T ret;
                    ss>>ret;
                    return ret;
                }
                return default_value;
            }
        };
        template<> std::string IniConfig::iniGet(std::string key, std::string default_value) {
            std::map<std::string, std::string>::const_iterator itr=ini.find(key);
            if(itr!=ini.end()) {
                return itr->second;
            }
            return default_value;
        }
    } // namespace ini_read

} // namespace peac_utils
