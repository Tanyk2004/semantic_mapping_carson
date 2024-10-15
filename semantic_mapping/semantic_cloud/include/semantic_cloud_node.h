#ifndef SEGMENTATION_DS_H
#define SEGMENTATION_DS_H
#define PCL_NO_PRECOMPILE

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <fstream>
#include <experimental/filesystem>

#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/common/transforms.h>

#include <set>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include <limits>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <chrono>
#include <thread>
#include <cstdlib> // For getting the home directory path

struct EIGEN_ALIGN16 SemanticPoint    // enforce SSE padding for correct memory alignment
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    int label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT (SemanticPoint,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (int, label, label))

using PointType = SemanticPoint;

namespace fs = std::experimental::filesystem;

std::string dataDirectory = fs::path(std::getenv("HOME")).string() + "/projects/catkin_ws_octomap/src/semantic_mapping/semantic_cloud/semantic_mapping_data/";
//std::string dataDirectory = "./semantic_mapping_data/";
std::vector<std::string> labels;

class SegmentationNetwork {
    protected:
        std::vector<std::string> allLabels;
        pcl::PolygonMesh meshes;
        pcl::PolygonMesh outputMesh;

        Eigen::Quaternionf* camRotations;
        Eigen::Vector3f* camTranslations;

        //Camera Model Parameters:
        double fx = 6.1244213867187500e+02;
        double fy = 6.1212139892578125e+02;
        double cx = 6.3851049804687500e+02;
        double cy = 3.6623443603515625e+02;

        cv::Mat K;
        cv::Mat D;
        cv::Mat map1, map2;

        pcl::PointCloud<PointType>::Ptr pts;
        pcl::KdTreeFLANN<PointType> kdtree;

        void parsePoseFile();

    public:
        SegmentationNetwork();
        ~SegmentationNetwork();

        void projectPoints(int frame_idx, cv::Mat labels_mat);

        void processFile(const fs::path& filePath);

        void saveData(int frame);

        cv::Mat undistort_img(cv::Mat in);
};


#endif