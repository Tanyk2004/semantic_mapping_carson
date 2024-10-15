#include <octomap_generator/next_best_view.h>
#include <octomap_generator/octomap_generator.h>

const float MAX_X = 0.5;
const float MAX_Y = 11.2;
const float MIN_X = -4.7;
const float MIN_Y = 3.0;

NextBestView::NextBestView(ros::NodeHandle& nh, OctomapGenerator* octomap_generator) : nh_(nh) {
    costmap.header.frame_id = "level_mux_map"; // Replace with your frame_id
    costmap.info.resolution = 0.05;    // Replace with your resolution
    costmap.info.width = 1721;         // Replace with your width
    costmap.info.height = 1106;        // Replace with your height
    costmap.info.origin.position.x = -75; // Replace with your origin
    costmap.info.origin.position.y = -25; // Replace with your origin
    costmap.data.resize(costmap.info.width * costmap.info.height, 0); // Initialize data with zeros
    localCostmap.info.resolution = 0.05;    // Replace with your resolution
    localCostmap.info.width = 160;         // Replace with your width
    localCostmap.info.height = 160;        // Replace with your height
    localCostmap.info.origin.position.x = 0; // Replace with your origin
    localCostmap.info.origin.position.y = 0; // Replace with your origin
    localCostmap.data.resize(localCostmap.info.width * localCostmap.info.height, 0); // Initialize data with zeros

    octomap_generator_ = octomap_generator;
    nh_ = nh;

    message_filters::Subscriber<map_msgs::OccupancyGridUpdate>* costmapSub = new message_filters::Subscriber<map_msgs::OccupancyGridUpdate> (nh_, "move_base/global_costmap/costmap_updates", 5);
    costmapSub->registerCallback(boost::bind(&NextBestView::costmapCallback, this, _1));
    message_filters::Subscriber<map_msgs::OccupancyGridUpdate>* localCostmapSub = new message_filters::Subscriber<map_msgs::OccupancyGridUpdate> (nh_, "/move_base/local_costmap/costmap_updates", 5);
    costmapSub->registerCallback(boost::bind(&NextBestView::localCostmapCallback, this, _1));
    markerPub = nh_.advertise<visualization_msgs::Marker>("canadite_pose_markers", 10);

    nbv_service_ = nh_.advertiseService("get_nbv", &NextBestView::nextBestPointReq, this);
}

void NextBestView::localCostmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
    for (int i = 0; i < msg->height; ++i) {
      for (int j = 0; j < msg->width; ++j) {
          int grid_x = msg->x + j;
          int grid_y = msg->y + i;

          // Check if the grid coordinates are within the local grid
          if (grid_x >= 0 && grid_x < localCostmap.info.width && grid_y >= 0 && grid_y < localCostmap.info.height) {
              int index = grid_y * localCostmap.info.width + grid_x;
              localCostmap.data[index] = msg->data[i * msg->width + j];
          }
      }
    }
} 

// Callback function to update the costmap
void NextBestView::costmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {

    cv::Mat costmap_img_ = cv::Mat(costmap.info.height, costmap.info.width, CV_8UC3);

    for (int i = 0; i < msg->height; ++i) {
      for (int j = 0; j < msg->width; ++j) {
          int grid_x = msg->x + j;
          int grid_y = msg->y + i;

          // Check if the grid coordinates are within the local grid
          if (grid_x >= 0 && grid_x < costmap.info.width && grid_y >= 0 && grid_y < costmap.info.height) {
              int index = grid_y * costmap.info.width + grid_x;
              costmap.data[index] = msg->data[i * msg->width + j];
          }
      }
    }

    /*for (int i = 0; i < costmap.info.height; ++i) {
        for (int j = 0; j < costmap.info.width; ++j) {
            int index = j + i * costmap.info.width;
            int8_t cost = costmap.data[index];

            // Map occupancy values to grayscale values (0-255)
            cv::Vec3b color = cv::Vec3b(255 - cost * 2, cost * 2, 0); // Custom color map

            costmap_img_.at<cv::Vec3b>(i, j) = color;
        }
    }

    // Draw a circle at the robot position
    cv::circle(costmap_img_, cv::Point(poseX, poseY), 5, cv::Scalar(0, 0, 255), -1);
    cv::resize(costmap_img_, costmap_img_, cv::Size(), 0.5, 0.5);
    cv::imshow("Costmap", costmap_img_);
    cv::waitKey(1);*/

    isCostmapReceived = true;
}

// Function to check if a given position is reachable in the costmap
bool NextBestView::isReachable(geometry_msgs::PoseStamped& pose) {
    // Check if the costmap has been received
    if (!isCostmapReceived) {
        ROS_WARN("Costmap not received yet.");
        return false;
    }

    /*if (pose.pose.position.x > MAX_X || pose.pose.position.x < MIN_X || pose.pose.position.y > MAX_Y || pose.pose.position.y < MIN_Y) {
        //ROS_WARN("Pose outside of map bounds.");
        return false;
    }*/

    // Convert pose to costmap coordinates
    double poseX = (pose.pose.position.x - costmap.info.origin.position.x) / costmap.info.resolution;
    double poseY = (pose.pose.position.y - costmap.info.origin.position.y) / costmap.info.resolution;

    double localOriginX = pose.pose.position.x - (160 * 0.05 / 2);
    double localOriginY = pose.pose.position.y - (160 * 0.05 / 2);
    double localPoseX = (pose.pose.position.x - localOriginX) / localCostmap.info.resolution;
    double localPoseY = (pose.pose.position.y - localOriginY) / localCostmap.info.resolution;

    // Retrieve the costmap dimensions
    int mapWidth = costmap.info.width;
    int mapHeight = costmap.info.height;

    // Check if the pose is within the costmap boundaries
    if (poseX < 0 || poseX >= mapWidth || poseY < 0 || poseY >= mapHeight) {
        ROS_WARN("Pose outside costmap boundaries.");
        return false;
    }

    // Index calculation for the 1D costmap array
    int mapIndex = int(poseX) + int(poseY) * costmap.info.width;
    int localMapIndex = int(localPoseX) + int(localPoseY) * localCostmap.info.width;

    // Retrieve the cost value at the specified index
    int costValue = costmap.data[mapIndex] + localCostmap.data[localMapIndex];

    // Set a threshold for considering the pose as reachable
    int freeThreshold = 30; // Change this threshold based on your costmap characteristics

    // Check if the cost value is below the threshold (considered free space)
    return costValue < freeThreshold;
}

bool NextBestView::nextBestPointReq(octomap_generator::NBV::Request &req, octomap_generator::NBV::Response &res) {
    // Retrieve the origin pose from the request
    geometry_msgs::PoseStamped origin = req.origin;

    robotPos = origin;

    std::cout << "Origin: " << origin.pose.position.x << ", " << origin.pose.position.y << std::endl;
    // Retrieve the next best point
    geometry_msgs::PoseStamped nbp = nextBestPoint(origin);

    // Return the next best point in the response
    res.nextBestPoint = nbp;

    return true;
}

// Function to sample a grid of points around a robot, apply random rotations, and check reachability
geometry_msgs::PoseStamped NextBestView::nextBestPoint(geometry_msgs::PoseStamped origin) {
    // Random number generator for rotations

    std::vector<ScoredPose> scoredPoses;

    int id = 0;

    // Sample a grid of points around the robot
    for (double x = -3; x <= 3; x += 1) {
        for (double y = -3; y <= 3; y += 1) {
            ScoredPose scoredPose = scorePose(x, y, origin, false, id++);
            if (scoredPose.score > 0) 
                scoredPoses.push_back(scoredPose);
        }
    }

    /*for (int i = 0; i < 4; i++) {
        double x = 0;
        double y = 0;
        //create pose stamped
        geometry_msgs::PoseStamped samplePose;
        // Set the new position
        while (x == 0 || !isReachable(samplePose)) {
            //random x in bounds
            x = (double)rand() / RAND_MAX;
            x = x * (MAX_X - MIN_X) + MIN_X;
            //random y
            y = (double)rand() / RAND_MAX;
            y = y * (MAX_Y - MIN_Y) + MIN_Y;
            samplePose.pose.position.x = x;
            samplePose.pose.position.y = y;
        }
        geometry_msgs::PoseStamped originPose;
        ScoredPose scoredPose = scorePose(x, y, originPose, true, id++);
        if (scoredPose.score > 0) 
            scoredPoses.push_back(scoredPose);
    }*/


    if (scoredPoses.size() == 0) {
        ROS_WARN("No reachable points found.");
        return origin;
    }

    // return point with highest score
    ScoredPose bestPose = scoredPoses[0];
    for (int i = 1; i < scoredPoses.size(); i++) {
        if (scoredPoses[i].score > bestPose.score) {
            bestPose = scoredPoses[i];
        }
    }
    std::cout << "Best pose: " << bestPose.pose.pose.position.x << ", " << bestPose.pose.pose.position.y << std::endl;
    std::cout << "Best score: " << bestPose.score << std::endl;
    return bestPose.pose;
}

ScoredPose NextBestView::scorePose(int x, int y, geometry_msgs::PoseStamped origin, bool distance_negative, int count) {
    geometry_msgs::PoseStamped samplePose;
    samplePose.header.frame_id = "/level_mux_map";

    // Set the new position
    samplePose.pose.position.x = origin.pose.position.x + x;
    samplePose.pose.position.y = origin.pose.position.y + y;
    std::cout << "Sample pose: " << samplePose.pose.position.x << ", " << samplePose.pose.position.y << std::endl;

    // Apply a random rotation
    double randomRotation = rand() / static_cast <double> (2.0 * M_PI);
    samplePose.pose.orientation = tf::createQuaternionMsgFromYaw(randomRotation);

    // Set up the marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "level_mux_map"; // Assuming your frame is "map", change if needed
    marker.header.stamp = ros::Time::now();
    marker.ns = "pose_markers";
    marker.id = count;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5; // Arrow shaft diameter
    marker.scale.y = 0.05; // Arrow head diameter
    marker.color.a = 1.0; // Alpha channel (1.0 is fully opaque)
    marker.color.r = 0.0; // Red color
    marker.color.g = 0.0; // Green color
    marker.color.b = 0.0; // Blue color

    ScoredPose scoredPose(samplePose, 0.0);
    marker.pose = samplePose.pose;
    // Check if the new position is reachable
    if (NextBestView::isReachable(samplePose)) {
        octomap::point3d pos = octomap::point3d(samplePose.pose.position.x, samplePose.pose.position.y, samplePose.pose.position.z);
        float distance = sqrt(pow(samplePose.pose.position.x - origin.pose.position.x, 2) + pow(samplePose.pose.position.y - origin.pose.position.y, 2));
        Eigen::Quaterniond quat(samplePose.pose.orientation.w,
                                    samplePose.pose.orientation.x,
                                    samplePose.pose.orientation.y,
                                    samplePose.pose.orientation.z);
        Eigen::Vector3d direction = quat * Eigen::Vector3d::UnitX();
        octomap::point3d dir = octomap::point3d(direction.x(), direction.y(), direction.z());
        int novelty = octomap_generator_->getPoseNovelty(this, pos, dir, 4);
        if (!distance_negative)
            scoredPose.score = novelty + (distance) * 25;
        else
            scoredPose.score = novelty - (distance) * 50;
        std::cout << "Score: " << novelty << std::endl;

        marker.color.r = std::min(1.0, ((scoredPose.score - 1000) / 3000)) * 255;
        marker.color.b = 255 - std::min(1.0, ((scoredPose.score - 1000) / 3000)) * 255;
        
    }

    markerPub.publish(marker);
    return scoredPose;
}

