#include <octomap_generator/octomap_generator.h>
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy
#include <vector>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap_types.h>
#include <octomap_generator/ocTreeKeyHash.h>
#include <octomap_generator/next_best_view.h>

// #include <octomap/OcTreeBaseImpl.hxx>
using octomap::OcTreeKey;

OctomapGenerator::OctomapGenerator(): octomap_(0.05), max_range_(1.), raycast_range_(1.)
{
  std::unordered_map<int, octomap::ColorOcTreeNode::Color> colorMap;
  std::cout << "octomap constructor" << std::endl;

  auto currentTime = std::chrono::system_clock::now();
  auto duration = currentTime.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  long currentTimestamp = seconds.count();
  start_time = currentTimestamp;
  std::cout << start_time << std::endl;

  srand((unsigned int)time(NULL)); //set seed
}

OctomapGenerator::~OctomapGenerator(){}

void OctomapGenerator::setUseSemanticColor(bool use)
{
  octomap_.setUseSemanticColor(use);
}

bool OctomapGenerator::isUseSemanticColor()
{
  return octomap_.isUseSemanticColor();
}

template <typename PointI, typename PointR>
class CustomPointToPointDistance : public pcl::registration::CorrespondenceEstimation<PointI, PointR> {
public:
    CustomPointToPointDistance() {}

    void determineCorrespondences(pcl::Correspondences& correspondences, double max_distance=std::numeric_limits< double >::max()) override {
        correspondences.clear();

        // Compute correspondences using your custom distance metric
        for (int i = 0; i < this->input_->points.size(); ++i) {
            int best_match = -1;
            double best_distance = std::numeric_limits<double>::max();
double best_score = std::numeric_limits<double>::max();
            for (int j = 0; j < this->target_->points.size(); ++j) {
                float importance_weight = 1;
                //if (this->input_->points[i].label == this->target_->points[j].label)
                    //importance_weight = 0.1;
                if (this->input_->points[i].label > 0 || this->target_->points[j].label > 0)
                    importance_weight = 0.5;

                double distance = pcl::squaredEuclideanDistance(this->input_->points[i], this->target_->points[j]);
                double score = distance * importance_weight;

                if (score < best_score && distance < max_distance) {
                    best_distance = distance;
                    best_score = score;
                    best_match = j;
                }
            }
            if (best_match != -1) {
                pcl::Correspondence corr(i, best_match, best_distance);
                correspondences.push_back(corr);
            }
        }
    }
};

// Define a custom warp function for LM optimization
template <typename PointI, typename PointR>
class TranslationOnly : public pcl::registration::WarpPointRigid3D<PointI, PointR>
{
public:
    // Override the operator to define your custom warp function
    virtual Eigen::Matrix4f operator() (const Eigen::Matrix4f& x) const {
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        // Implement your custom transformation here
        // For example, if you want to perform translation-only optimization:
        transformation_matrix.block<3, 1>(0, 3) = x.block<3, 1>(0, 0);
        return transformation_matrix;
    }
};

void OctomapGenerator::registerPointCloud(PCLSemanticPoint::Ptr cloud) {
  
  //pcl::registration::WarpPointRigid3D<SemanticPoint, SemanticPoint>::Ptr warp_fcn (new pcl::registration::WarpPointRigid3D<SemanticPoint, SemanticPoint>);
  pcl::registration::TransformationEstimationLM<SemanticPoint, SemanticPoint>::Ptr te (new pcl::registration::TransformationEstimationLM<SemanticPoint, SemanticPoint>);
  TranslationOnly<SemanticPoint, SemanticPoint>::Ptr warp_fcn (new TranslationOnly<SemanticPoint, SemanticPoint>);
  te->setWarpFunction (warp_fcn);

  PCLSemanticPoint::Ptr ground_filtered (new PCLSemanticPoint);
  for(PCLSemanticPoint::const_iterator it = cloud->begin(); it != cloud->end(); ++it)
  {
    // Check if the point is invalid
    if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
    {
        if (it->z > 0.1)
            ground_filtered->push_back(*it);
    }
  }

  pcl::PointCloud<SemanticPoint>::Ptr totalCloud (new pcl::PointCloud<SemanticPoint>);
  std::list<pcl::PointCloud<SemanticPoint>::Ptr>::iterator it;
  for (it = previousClouds.begin(); it != previousClouds.end(); it++)
	{
		*totalCloud += *(*it);
	}
  

  pcl::io::savePLYFileASCII("input_cloud.ply", *cloud);
  pcl::io::savePLYFileASCII("target_cloud.ply", *totalCloud);

  pcl::IterativeClosestPoint<SemanticPoint, SemanticPoint> icp;
  icp.setInputSource(ground_filtered);
  icp.setInputTarget(totalCloud);
  // Set ICP parameters
  icp.setMaxCorrespondenceDistance(1);  // Adjust as needed
  icp.setMaximumIterations(50);          // Adjust as needed
  icp.setTransformationEpsilon(1e-8);     // Adjust as needed
  // Pass the TransformationEstimation object to the ICP algorithm
  icp.setTransformationEstimation(te);
  // Set the correspondence distance calculation object
  CustomPointToPointDistance<SemanticPoint, SemanticPoint>::Ptr distance (new CustomPointToPointDistance<SemanticPoint, SemanticPoint>);
  //icp.setCorrespondenceEstimation(distance);

  PCLSemanticPoint::Ptr registered_cloud(new PCLSemanticPoint);
  icp.align(*registered_cloud);
  std::cout << icp.getFitnessScore() << std::endl;
  if (icp.hasConverged()) {
    pcl::io::savePLYFileASCII("registered_cloud.ply", *cloud);
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    // Apply the transformation to the source cloud
    pcl::transformPointCloud(*cloud, *cloud, transformation);
  }

  previousClouds.push_front(ground_filtered);
  if (previousClouds.size() > 3)
    previousClouds.pop_back();

  //probably a new area that doesn't correspond, don't risk trying to fit it somewhere where it doesn't belong
  std::cout << "ICP didn't converge, adding raw points" << std::endl;
  return;
}

pcl::PointCloud<SemanticPoint>::Ptr downsampleMaxLabel(const pcl::PointCloud<SemanticPoint>::Ptr& input_cloud, float leaf_size)
{
    pcl::PointCloud<SemanticPoint>::Ptr downsampled_cloud(new pcl::PointCloud<SemanticPoint>);
    std::unordered_map<std::string, std::vector<SemanticPoint>> voxel_map;

    for (const SemanticPoint& point : input_cloud->points)
    {
        int voxel_x = static_cast<int>(point.x / leaf_size);
        int voxel_y = static_cast<int>(point.y / leaf_size);
        int voxel_z = static_cast<int>(point.z / leaf_size);

        // Create a unique key for the voxel based on its coordinates
        std::string voxel_key = std::to_string(voxel_x) + "_" + std::to_string(voxel_y) + "_" + std::to_string(voxel_z);

        // Add the point to the list of points in the voxel
        voxel_map[voxel_key].push_back(point);
    }

    // Populate the downsampled point cloud with voxel centroids
    for (const auto& entry : voxel_map)
    {
        SemanticPoint point;
        std::string voxel_key = entry.first;
        const std::vector<SemanticPoint>& points_in_voxel = entry.second;

        // Extract voxel coordinates from the key
        sscanf(voxel_key.c_str(), "%d_%d_%d", &point.x, &point.y, &point.z);

        // Calculate the centroid of points in the voxel
        float sum_x = 0.0;
        float sum_y = 0.0;
        float sum_z = 0.0;
        std::unordered_map<int, int> label_count;
        int num_points = static_cast<int>(points_in_voxel.size());

        for (const SemanticPoint& p : points_in_voxel)
        {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
            label_count[p.label]++;
        }

        int max_label = 0;
        int max_count = 0;
        for (const auto& entry : label_count)
        {
            if (entry.second > max_count)
            {
                max_label = entry.first;
                max_count = entry.second;
            }
        }

        point.x = sum_x / num_points;
        point.y = sum_y / num_points;
        point.z = sum_z / num_points;
        point.label = max_label;

        downsampled_cloud->points.push_back(point);
    }

    downsampled_cloud->width = static_cast<uint32_t>(downsampled_cloud->points.size());
    downsampled_cloud->height = 1;

    return downsampled_cloud;
}

void OctomapGenerator::clusterObjects(PCLSemanticPoint::Ptr pts) {
    std::cout << "Finding Object Clusters" << std::endl;

    std::list<ObjectInstance> objectList;

    pcl::PointCloud<SemanticPoint>::Ptr labeledCloud(new pcl::PointCloud<SemanticPoint>);

    pcl::ConditionAnd<SemanticPoint>::Ptr condition(new pcl::ConditionAnd<SemanticPoint>());
    condition->addComparison(pcl::FieldComparison<SemanticPoint>::Ptr(
        new pcl::FieldComparison<SemanticPoint>("label", pcl::ComparisonOps::GT, 0)));

    // filter out points with label 0
    pcl::ConditionalRemoval<SemanticPoint> filter;
    filter.setInputCloud(pts);
    filter.setCondition(condition);
    filter.setKeepOrganized(false);  // Preserve the structure of the cloud
    filter.filter(*labeledCloud);

    std::function<bool(const SemanticPoint&, const SemanticPoint&, float)> cluster_func =
      [](const SemanticPoint& p1, const SemanticPoint& p2, float dist) {
        // Check if the points have the same label and are within the distance threshold
        return p1.label == p2.label &&
               dist <= 0.25;
      };

    // Euclidean cluster extraction
    pcl::search::KdTree<SemanticPoint>::Ptr kd_tree(new pcl::search::KdTree<SemanticPoint>);
    kd_tree->setInputCloud(labeledCloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::ConditionalEuclideanClustering<SemanticPoint> euclidean_cluster;
    euclidean_cluster.setClusterTolerance(0.1);  // Set the cluster tolerance
    euclidean_cluster.setMinClusterSize(5);     // Set the minimum cluster size
    euclidean_cluster.setMaxClusterSize(10000);   // Set the maximum cluster size
    euclidean_cluster.setSearchMethod(kd_tree);
    euclidean_cluster.setInputCloud(labeledCloud);
    euclidean_cluster.setConditionFunction(cluster_func);
    euclidean_cluster.segment(cluster_indices);

    // Create a PCL visualizer
    //pcl::visualization::PCLVisualizer viewer("Object Meshes");

    // Set the background color to white
    //viewer.setBackgroundColor(1.0, 1.0, 1.0);

    // Add the filtered point cloud to the viewer
    //viewer.addPointCloud(pts, "cloud");

    for (std::size_t i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<SemanticPoint>::Ptr objectCloud(new pcl::PointCloud<SemanticPoint>);
        if (cluster_indices[i].indices.size() == 0)
            continue;

        int label_ind = labeledCloud->at(cluster_indices[i].indices[0]).label - 1;
        for (std::size_t j = 0; j < cluster_indices[i].indices.size(); ++j)
        {
            objectCloud->push_back(labeledCloud->at(cluster_indices[i].indices[j]));
        }

        SemanticPoint minPoint, maxPoint;
        pcl::getMinMax3D(*objectCloud, minPoint, maxPoint);

        octomap::point3d pos = octomap::point3d((minPoint.x + maxPoint.x) / 2, (minPoint.y + maxPoint.y) / 2, (minPoint.z + maxPoint.z) / 2);
        octomap::point3d size = octomap::point3d(maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z);
        ObjectInstance obj(label_ind, pos, size);
        objectList.push_back(obj);

        std:cout << "Cluster Bounds: " << minPoint.x << " " << minPoint.y << " " << minPoint.z << " " << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z << std::endl;

        // Print the coordinates of the bounding box
        std::cout << "Label: " << label_ind << std::endl;
    }

    // Spin the viewer
    //viewer.spin();

    return objectList;
}

octomap::point3d projectPointIntoImage(const octomap::point3d& point, const Eigen::Matrix4f& sensorToWorld) {
    Eigen::Matrix<float, 4, 4> Projection;
      Projection << 6.1244213867187500e+02, 0., 6.3851049804687500e+02, 0.,
      0., 6.1212139892578125e+02, 3.6623443603515625e+02, 0.,
      0., 0., 1., 0., 
      0, 0, 0, 1;
    
    Eigen::Vector4f point3d(point.x(), point.y(), point.z(), 1);
    Eigen::Vector4f point2d = projection * sensorToWorld * point3d;
    return octomap::point3d(point2d[0], point2d[1], point2d[2]);
}

void distance (octomap::point3d& p1, octomap::point3d& p2) {
    return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y()) + (p1.z() - p2.z()) * (p1.z() - p2.z()));
}

void OctomapGenerator::matchObjects(std::list<ObjectInstance> newObjects) {
    std::cout << "Matching Objects" << std::endl;

    for (auto it = newObjects.begin(); it != newObjects.end(); it++) {
        ObjectInstance a = *it;
        float minDist = 1000000;
        ObjectInstance matched;
        for (auto it2 = objects.begin(); it2 != objects.end(); it2++) {
            ObjectInstance b = *it2;

            if (b.label != a.label)
                continue;

            float dist = distance(a.getPosition(), b.getPosition());
            if (dist < minDist) {
                minDist = dist;
                matched = b;
            }
        }

        if (matched == NULL) {
            //add
            objects.push_back(a);
        }
        else {
            //update matched obj
            matched.setTimestamp(start_time);
            matched.setPosition(a.getPosition());
            matched.setSize(a.getSize());
        }
    }

    //std::list<ObjectInstance> objInFrame;
    for (auto it = objects.begin(); it != objects.end(); it++) {
        ObjectInstance obj = *it;

        if (obj.getTimestamp() == 0) {
            objInFrame.push_back(obj);
        }
    }
}

void OctomapGenerator::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld)
{
  std::cout << "callback" << std::endl;

  // Convert to PCL pointcloud
  PCLSemanticPoint::Ptr pcl_cloud (new PCLSemanticPoint);
  pcl::fromPCLPointCloud2(*cloud, *pcl_cloud);

  // Transform coordinate
  pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, sensorToWorld);

  // Voxel filter to down sample the point cloud
  // Create the filtering object
  std::cout << "Voxel unfiltered cloud size: "<< pcl_cloud->size() << std::endl;
  pcl::PointCloud<SemanticPoint>::Ptr cloud_filtered = downsampleMaxLabel(pcl_cloud, octomap_.getResolution());
  std::cout << "Voxel filtered cloud size: "<< cloud_filtered->size() << std::endl;

  std::list<ObjectInstance> newObjects = clusterObjects(cloud_filtered);
  matchObjects(newObjects);

  if (previousClouds.size() > 0)
    registerPointCloud(cloud_filtered);

  PCLSemanticPoint::Ptr close_cloud (new PCLSemanticPoint);

  std::cout << "Registered cloud size" << std::endl;

  //tf::Vector3 originTf = sensorToWorldTf.getOrigin();
  octomap::point3d origin(static_cast<float>(sensorToWorld(0,3)),static_cast<float>(sensorToWorld(1,3)),static_cast<float>(sensorToWorld(2,3)));
  octomap::Pointcloud raycast_cloud; // Point cloud to be inserted with ray casting
  int endpoint_count = 0; // total number of endpoints inserted
  for(PCLSemanticPoint::const_iterator it = cloud_filtered->begin(); it != cloud_filtered->end(); ++it)
  {
    // Check if the point is invalid
    if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
    {
      float dist = sqrt((it->x - origin.x())*(it->x - origin.x()) + (it->y - origin.y())*(it->y - origin.y()) + (it->z - origin.z())*(it->z - origin.z()));
      // Check if the point is in the ray casting range
      if(dist <= raycast_range_) // Add to a point cloud and do ray casting later all together
      {
        raycast_cloud.push_back(it->x, it->y, it->z);
      }
      else // otherwise update the occupancy of node and transfer the point to the raycast range
      {
        octomap::point3d direction = (octomap::point3d(it->x, it->y, it->z) - origin).normalized ();
        octomap::point3d new_end = origin + direction * (raycast_range_ + octomap_.getResolution()*2);
        raycast_cloud.push_back(new_end);
        if(dist <= max_range_)
          octomap_.updateNode(it->x, it->y, it->z, true, false); // use lazy_eval, run updateInnerOccupancy() when done
      }
      if (dist <= 6) {
        close_cloud->push_back(*it);
      }
      endpoint_count++;
    }
  }
  // Do ray casting for points in raycast_range_
  if(raycast_cloud.size() > 0)
    octomap_.insertPointCloud(raycast_cloud, origin, raycast_range_, false, true);  // use lazy_eval, run updateInnerOccupancy() when done, use discretize to downsample cloud
  
  // updates inner node occupancy and colors
  if(endpoint_count > 0)
    octomap_.updateInnerOccupancy();

  // Update colors and semantics, differs between templates
  updateColorAndSemantics(close_cloud);

  //cluster objects
  std::vector<std::pair<Point3D, Point3D>> cluster_min_max_points;
  std::vector<int> label_set;
  std::unordered_set<std::string> visitedCoords;

  for (auto it = octomap_.begin_leafs(); it != octomap_.end_leafs(); ++it) {
    SemanticsOcTreeNodeMaxLabel node = *it;
    if (node.getSemantics().getLabel() > 0 && visitedCoords.find(std::to_string(it.getX()) + "_" + std::to_string(it.getY()) + "_" + std::to_string(it.getZ())) == visitedCoords.end()) {
      std::vector<Point3D> cluster_points;
      std::vector<Point3D> min_max_points(2, {DBL_MAX, DBL_MAX, DBL_MAX});
      min_max_points[1] = {-1.0 * DBL_MAX, -1.0 * DBL_MAX, -1.0 * DBL_MAX};
      octomap::OcTreeKey key = it.getKey();
      int label = node.getSemantics().getLabel();
      SemanticsOcTreeNodeMaxLabel* node = octomap_.search(key);
      // std::cout << "key:  " << key[0] << " " << key[1] << " " << key[2] << std::endl;
      // std::cout << node->getSemantics().getLabel() << std::endl;
      dfsTraversal(&octomap_, key, cluster_points, min_max_points, &visitedCoords, label);
      
      if (!cluster_points.empty()) {
          label_set.push_back(label);
          //std::cout << "min point in dfs: " << min_max_points[0].x << min_max_points[0].y << min_max_points[0].z << std::endl;
          //std::cout << "max point in dfs: " << min_max_points[1].x << min_max_points[1].y << min_max_points[1].z << std::endl;
          cluster_min_max_points.push_back(std::make_pair(min_max_points[0], min_max_points[1]));
      }
    }
  }

  //add count to file
  std::string file_name = "/root/projects/catkin_ws_octomap/src/semantic_mapping/cluster_points" + std::to_string(count++) + ".txt"; 

  std::ofstream outputFile(file_name);
  std::ofstream outputFileTotal("/root/projects/catkin_ws_octomap/src/semantic_mapping/cluster_points.txt");
  if (outputFile.is_open()) {
      for (size_t i = 0; i < cluster_min_max_points.size(); ++i) {
          /*std::cout << "Cluster " << i + 1 << " Label: " << label_set[i];
          std::cout << "Cluster " << i + 1 << " Min Point (XYZ): (" << cluster_min_max_points[i].first.x
                    << ", " << cluster_min_max_points[i].first.y << ", " << cluster_min_max_points[i].first.z << ")\n";
          std::cout << "Cluster " << i + 1 << " Max Point (XYZ): (" << cluster_min_max_points[i].second.x
                    << ", " << cluster_min_max_points[i].second.y << ", " << cluster_min_max_points[i].second.z << ")\n";*/
          outputFile << label_set[i] <<  " " << cluster_min_max_points[i].first.x << " " << cluster_min_max_points[i].first.y << " " << cluster_min_max_points[i].first.z
                    << " " << cluster_min_max_points[i].second.x << " " << cluster_min_max_points[i].second.y << " " << cluster_min_max_points[i].second.z << "\n";
          outputFileTotal << label_set[i] <<  " " << cluster_min_max_points[i].first.x << " " << cluster_min_max_points[i].first.y << " " << cluster_min_max_points[i].first.z
                    << " " << cluster_min_max_points[i].second.x << " " << cluster_min_max_points[i].second.y << " " << cluster_min_max_points[i].second.z << "\n";
      }
      outputFile.close();
      outputFileTotal.close();
      std::cout << "Results saved to cluster_points.txt" << std::endl;
  } else {
      std::cerr << "Unable to open the output file." << std::endl;
  }

}

// A helper function to perform DFS traversal
void OctomapGenerator::dfsTraversal(SemanticsOctreeMaxLabel* tree, octomap::OcTreeKey key, std::vector<Point3D>& points, 
                  std::vector<Point3D>& min_max_points, std::unordered_set<std::string>* visitedCoords, int target_label) {
  // std::cout << "key:  " << key[0] << " " << key[1] << " " << key[2] << std::endl;
  SemanticsOcTreeNodeMaxLabel* node = tree->search(key);
  octomap::point3d currCoord = octomap_.keyToCoord(key);
  std::string currCoordStr = std::to_string(currCoord.x()) + "_" + std::to_string(currCoord.y()) + "_" + std::to_string(currCoord.z());
  // std::cout << node->getSemantics().getLabel() << " " << target_label << std::endl;
  if (node != nullptr && node->getSemantics().getLabel() == target_label && visitedCoords->find(currCoordStr) == visitedCoords->end()) {
      // Add the current point to the cluster
      struct Point3D currPoint;
      currPoint.x = static_cast<double>(currCoord.x());
      currPoint.y = static_cast<double>(currCoord.y());
      currPoint.z = static_cast<double>(currCoord.z());
      points.push_back(currPoint);
      //std::cout << "X: " << currCoord.x() << " Y: " << currCoord.y() << " Z: " << currCoord.z() << std::endl;
      //std::cout << "X: " << currPoint.x << " Y: " << currPoint.y << " Z: " << currPoint.z << std::endl;
      // std::cout << "DFS: " << target_label << " " << currCoordStr << std::endl;
      
      // Update min and max points
      // Point3D min_point = min_max_points[0];
      // Point3D max_point = min_max_points[1];
      min_max_points[0].x = std::min(min_max_points[0].x, currPoint.x);
      min_max_points[0].y = std::min(min_max_points[0].y, currPoint.y);
      min_max_points[0].z = std::min(min_max_points[0].z, currPoint.z);

      min_max_points[1].x = std::max(min_max_points[1].x, currPoint.x);
      min_max_points[1].y = std::max(min_max_points[1].y, currPoint.y);
      min_max_points[1].z = std::max(min_max_points[1].z, currPoint.z);
      //std::cout << "min point in dfs: " << min_max_points[0].x << min_max_points[0].y << min_max_points[0].z << std::endl;
      //std::cout << "max point in dfs: " << min_max_points[1].x << min_max_points[1].y << min_max_points[1].z << std::endl;

      // Mark the node as visited
      visitedCoords->insert(currCoordStr);
      
      int neighborOffsets[26][3] = {
      {0, 0, 1}, {0, 0, -1}, // Front and back neighbors
      {0, 1, 0}, {0, -1, 0}, // Top and bottom neighbors
      {1, 0, 0}, {-1, 0, 0}, // Left and right neighbors
      {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0}, // Diagonal neighbors in XY plane
      {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1}, // Diagonal neighbors in XZ plane
      {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}, // Diagonal neighbors in YZ plane
      {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1} // Corner neighbors
      };

      // Iterate through neighboring voxels
      for (int i = 0; i < 26; ++i) {
        octomap::OcTreeKey neighborKey = key;
        neighborKey[0] += neighborOffsets[i][0];
        neighborKey[1] += neighborOffsets[i][1];
        neighborKey[2] += neighborOffsets[i][2];
        dfsTraversal(tree, neighborKey, points, min_max_points, visitedCoords, target_label);
      }
    }
}
                  

void OctomapGenerator::updateColorAndSemantics(PCLSemanticPoint::Ptr pcl_cloud)
{
  //stores histogram of labels for each voxel
  std::unordered_map<octomap::point3d, std::unordered_map<int, int>> labelsMap;

  std::cout << pcl_cloud->size() << std::endl;

  std::set<std::string> modified_voxels = std::set<std::string>();
  for(PCLSemanticPoint::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
  {
    if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
    {
      octomap::SemanticsMaxLabel sem;
      sem.confidence = 1;
      sem.label = it->label;
      octomap::OcTreeKey key;
      octomap_.coordToKeyChecked(it->x, it->y, it->z, key);
      octomap::point3d currCoord = octomap_.keyToCoord(key);
      std::string currCoordStr = std::to_string(currCoord.x()) + "_" + std::to_string(currCoord.y()) + "_" + std::to_string(currCoord.z());
      modified_voxels.insert(currCoordStr);
      octomap_.updateNodeSemantics(key, sem);
    }
  }

  std::cout << "modified voxels: " << modified_voxels.size() << std::endl;

  int sameCount = 0;
  int diffCount = 0;
  for (std::set<std::string>::iterator it = modified_voxels.begin(); it != modified_voxels.end(); ++it) {
    std::string coord_str = *it;
    float x, y, z;
    sscanf(coord_str.c_str(), "%f_%f_%f", &x, &y, &z);
    octomap::OcTreeKey key;
    octomap_.coordToKeyChecked(x, y, z, key);
    SemanticsOcTreeNodeMaxLabel* node = octomap_.search(key);
    if (node != NULL) {
      int prev = node->getSemantics().label;
      //std::cout << "prev: " << node->getSemantics().label << std::endl;
      octomap::SemanticsMaxLabel sem = octomap::SemanticsMaxLabel::selectMaxLabel(node->getSemantics(), colorMap);
      node->setSemantics(sem);
      //std::cout << "after: " << node->getSemantics().label << std::endl;
      //std::cout << node->getSemantics().getTimestamp() << std::endl;
      if (node->getSemantics().label == prev) {
        sameCount++;
      }
      else {
        diffCount++;
      }
    }
  }

  //std::cout << "same: " << sameCount << " diff: " << diffCount << std::endl;

  /*SemanticsOcTreeNodeMaxLabel* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
  std::cout << "Example octree node: " << std::endl;
  std::cout << "Color: " << node->getColor()<< std::endl;
  std::cout << "Semantics: " << node->getSemantics() << std::endl;*/
}

int OctomapGenerator::getPoseNovelty(NextBestView* nbv, octomap::point3d& origin, octomap::point3d& direction, float max_range) {
  Eigen::Matrix<float, 4, 4> Projection;
      Projection << 6.1244213867187500e+02, 0., 6.3851049804687500e+02, 0.,
      0., 6.1212139892578125e+02, 3.6623443603515625e+02, 0.,
      0., 0., 1., 0., 
      0, 0, 0, 1;

  Eigen::Matrix<float, 4, 4> projectionInverse = Projection.inverse();

  int noveltyScore = 0;
  int free_cells = 0;
  std::set <std::string> countedCells;
  for (int u=200; u < 1080; u += 20) {
    for (int v=int(720/2); v < 720; v += 20) {
        float depthValue = max_range;

        //transform to 3D relative frame
        Eigen::Vector4f imagePoint(u, v, 1.0, 1 / depthValue);
        Eigen::Vector4f world_point = depthValue * projectionInverse * imagePoint;
        octomap::point3d end_point;
        octomap_.castRay(origin, direction, end_point, true, max_range);

        auto currentTime = std::chrono::system_clock::now();
        auto duration = currentTime.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        long currentTimestamp = seconds.count();

        // count free voxels between origin and endpoints
        octomap::KeyRay keyRay;
        if (octomap_.computeRayKeys(origin, end_point, keyRay)) {
          for (octomap::KeyRay::iterator it = keyRay.begin(); it != keyRay.end(); ++it) {
            SemanticsOcTreeNodeMaxLabel* node = octomap_.search(*it);
            octomap::point3d currCoord = octomap_.keyToCoord(*it);
            std::string currCoordStr = std::to_string(currCoord.x()) + "_" + std::to_string(currCoord.y()) + "_" + std::to_string(currCoord.z());
            if (node == NULL || !node->getSemantics().isSemanticsSet()) {
              geometry_msgs::PoseStamped pose;
              pose.pose.position.x = currCoord.x();
              pose.pose.position.y = currCoord.y();
              pose.pose.position.z = currCoord.z();
              if (nbv->isReachable(pose)) {
                if (countedCells.insert(currCoordStr).second) {
                  noveltyScore += currentTimestamp - start_time;
                  free_cells++;
                }
              }
              else
                break;
            }
            else if (node->getOccupancy() < 0.5) {
              if (countedCells.insert(currCoordStr).second) {
                noveltyScore += currentTimestamp - node->getSemantics().getTimestamp();
                free_cells++;
              }
            }
            else
              break;

          }
        }
    }
  }

  //noveltyScore /= free_cells;
  std::cout << free_cells / 2 << " " << noveltyScore << std::endl;

  return noveltyScore;// + free_cells / 2;
}

bool OctomapGenerator::save(const char* filename) const
{
  std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
  if (outfile.is_open()){
    std::cout << "Writing octomap to " << filename << std::endl;
    octomap_.write(outfile);
    outfile.close();
    std::cout << "Color tree written " << filename << std::endl;
    return true;
  }
  else {
    std::cout << "Could not open " << filename  << " for writing" << std::endl;
    return false;
  }
}
