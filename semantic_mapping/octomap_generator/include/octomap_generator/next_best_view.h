#ifndef NEXT_BEST_VIEW_H
#define NEXT_BEST_VIEW_H

#include <ros/ros.h>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_generator/octomap_generator.h>
#include <octomap_generator/NBV.h>
#include <random>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <message_filters/subscriber.h>

class OctomapGenerator;

class ScoredPose {
public:
    geometry_msgs::PoseStamped pose;
    double score;

    ScoredPose(const geometry_msgs::PoseStamped& pose, double score) : pose(pose), score(score) {}
};

class NextBestView {
    public:
        NextBestView(ros::NodeHandle& nh, OctomapGenerator* octomap_generator);

        void costmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
        void localCostmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
        bool isReachable(geometry_msgs::PoseStamped& pose);
        bool nextBestPointReq(octomap_generator::NBV::Request &req, octomap_generator::NBV::Response &res);
        geometry_msgs::PoseStamped nextBestPoint(geometry_msgs::PoseStamped origin);
        ScoredPose scorePose(int x, int y, geometry_msgs::PoseStamped origin, bool distance_negative, int count);

    protected:
        ros::NodeHandle nh_;
        octomap_msgs::Octomap map_msg_; ///<ROS octomap message
        nav_msgs::OccupancyGrid costmap;
        nav_msgs::OccupancyGrid localCostmap;
        bool isCostmapReceived;
        ros::Publisher markerPub;
        ros::ServiceServer nbv_service_;
        geometry_msgs::PoseStamped robotPos;
        OctomapGenerator* octomap_generator_;
};

#endif
