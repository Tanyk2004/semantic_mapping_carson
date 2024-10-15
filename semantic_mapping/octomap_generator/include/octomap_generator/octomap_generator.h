#ifndef OCTOMAP_GENERATOR_H
#define OCTOMAP_GENERATOR_H

#include <semantics_octree/semantics_octree.h>
#include <semantics_octree/semantics_max.h>
#include <semantics_point_type/semantics_point_type.h>
#include <octomap_generator/octomap_generator_base.h>
#include <octomap_generator/next_best_view.h>
#include <unordered_map>
#include <unordered_set>
#include <octomap/ColorOcTree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h> 
#include <list>
#include <chrono>
#include <object_instance.h>

typedef pcl::PointCloud<SemanticPoint> PCLSemanticPoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLColor;

typedef octomap::ColorOcTree ColorOcTree;
typedef octomap::SemanticsOcTree<octomap::SemanticsMaxLabel> SemanticsOctreeMaxLabel;
typedef octomap::SemanticsOcTreeNode<octomap::SemanticsMaxLabel> SemanticsOcTreeNodeMaxLabel;

class NextBestView;

/**
 * Templated octomap generator to generate a color octree or a semantic octree (with different fusion methods)
 * See base class for details
 * \author Xuan Zhang
 * \data Mai-July 2018
 */
class OctomapGenerator: public OctomapGeneratorBase
{
  public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGenerator();

    virtual ~OctomapGenerator();

    virtual void setMaxRange(float max_range){max_range_ = max_range;}

    virtual void setRayCastRange(float raycast_range){raycast_range_ = raycast_range;}

    virtual void setClampingThresMin(float clamping_thres_min)
    {
      octomap_.setClampingThresMin(clamping_thres_min);
    }

    virtual void setClampingThresMax(float clamping_thres_max)
    {
      octomap_.setClampingThresMax(clamping_thres_max);
    }

    virtual void setResolution(float resolution)
    {
      octomap_.setResolution(resolution);
    }

    virtual void setOccupancyThres(float occupancy_thres)
    {
      octomap_.setOccupancyThres(occupancy_thres);
    }

    virtual void setProbHit(float prob_hit)
    {
      octomap_.setProbHit(prob_hit);
    }

    virtual void setProbMiss(float prob_miss)
    {
      octomap_.setProbMiss(prob_miss);
    }



    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld);

    virtual void setUseSemanticColor(bool use);

    virtual bool isUseSemanticColor();

    virtual octomap::AbstractOcTree* getOctree(){return &octomap_;}

    struct Point3D {
      double x, y, z;
    };

    void dfsTraversal(SemanticsOctreeMaxLabel* tree, octomap::OcTreeKey key, std::vector<Point3D>& points, 
                  std::vector<Point3D>& min_max_points, std::unordered_set<std::string>* visitedCoords, int target_label);

    int getPoseNovelty(NextBestView* nbv, octomap::point3d& origin, octomap::point3d& direction, float max_range);

    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    virtual bool save(const char* filename) const;

    void registerPointCloud(PCLSemanticPoint::Ptr cloud);

    std::unordered_map<int, octomap::ColorOcTreeNode::Color> colorMap;
    std::list<PCLSemanticPoint::Ptr> previousClouds;
    std::list<ObjectInstance> objects;
    long start_time;

  protected:
    SemanticsOctreeMaxLabel octomap_; ///<Templated octree instance
    float max_range_; ///<Max range for points to be inserted into octomap
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space
    void updateColorAndSemantics(PCLSemanticPoint::Ptr pcl_cloud);
    int count;

};
#endif//OCTOMAP_GENERATOR
