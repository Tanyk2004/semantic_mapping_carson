#include "semantic_cloud_node.h"
#include <pcl/io/ply_io.h> 

ros::Publisher cloudPub; 

SegmentationNetwork::SegmentationNetwork() : pts(new pcl::PointCloud<PointType>) {
    //Initalize Camera Model:
    K = cv::Mat::eye(3, 3, cv::DataType<float>::type);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    D = cv::Mat(1, 8, cv::DataType<float>::type);

    D.at<float>(0, 0) = 0.2063539773225784;
    D.at<float>(0, 1) = -2.55633544921875;
    D.at<float>(0, 2) = 0.0003513785777613521;
    D.at<float>(0, 3) = -0.0002052536583505571;
    D.at<float>(0, 4) = 1.615918040275574;
    D.at<float>(0, 5) = 0.08677487820386887;
    D.at<float>(0, 6) = -2.35876727104187;
    D.at<float>(0, 7) = 1.526051759719849;

    cv::initUndistortRectifyMap(K, D, cv::Mat(), K, cv::Size(1280, 720), CV_32FC1, map1, map2);
}

cv::Mat SegmentationNetwork::undistort_img(cv::Mat in) {
    cv::Mat out;
    cv::remap(in, out, map1, map2, cv::INTER_NEAREST);
    return out;
}

void SegmentationNetwork::projectPoints(int frame_idx, cv::Mat labels_mat) {
    Eigen::Matrix<float, 4, 4> Projection;
        Projection << 6.1244213867187500e+02, 0., 6.3851049804687500e+02, 0.,
        0., 6.1212139892578125e+02, 3.6623443603515625e+02, 0.,
        0., 0., 1., 0., 
        0, 0, 0, 1;

    Eigen::Matrix<float, 4, 4> projectionInverse = Projection.inverse();

    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    std::cout << "Starting " << frame_idx << " image" << std::endl;
    std::string depthimagePath = dataDirectory + "depth/" + std::to_string(frame_idx) + ".png";
    //std::string rgbimagePath = dataDirectory + "images/" + std::to_string(frame_idx) + ".png";
    cv::Mat depthImage = cv::imread(depthimagePath, cv::IMREAD_UNCHANGED);
    depthImage = undistort_img(depthImage);
    //cv::Mat rgbImage = cv::imread(rgbimagePath, cv::IMREAD_COLOR);
    //rgbImage = undistort_img(rgbImage);
    labels_mat.convertTo(labels_mat, CV_8UC1);

    // Check if the image was loaded successfully
    if (depthImage.empty()) {
        std::cout << "Failed to load the depth image.\n";
        return;
    }
    
    cv::Mat depthFloat;
    depthImage.convertTo(depthFloat, CV_32F);

    //Eigen::Vector3f translation = camTranslations[frame_idx];
    //Eigen::Quaternionf rotation = camRotations[frame_idx];

    //Eigen::Matrix4f camera_pose_matrix = Eigen::Matrix4f::Identity();
    //camera_pose_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    //camera_pose_matrix.block<3, 1>(0, 3) = translation;

    pcl::PointCloud<PointType>::Ptr newPointcloud (new pcl::PointCloud<PointType>);

    float depthValue = 0.0;
    for (int u=0; u < depthFloat.cols; u += 3) {
        for (int v=int(depthFloat.rows/2); v < depthFloat.rows; v += 3) {
            depthValue = depthFloat.at<float>(v, u) * 0.001;
            //cout << depthValue << endl;

            if (depthValue == 0.0)
                depthValue = 8;

            //transform to 3D relative frame
            Eigen::Vector4f imagePoint(u, v, 1.0, 1 / depthValue);
            Eigen::Vector4f world_point = depthValue * projectionInverse * imagePoint;

            //add point to new point cloud piece
            PointType newPoint;
            newPoint.x = world_point.x();
            newPoint.y = world_point.y();
            newPoint.z = world_point.z();
            /*newPoint.r = rgbImage.at<cv::Vec3b>(v, u)[2];
            newPoint.g = rgbImage.at<cv::Vec3b>(v, u)[1];
            newPoint.b = rgbImage.at<cv::Vec3b>(v, u)[0];*/
            newPoint.label = labels_mat.at<uint8_t>(v, u);

            if (isnan(newPoint.x) || isnan(newPoint.y) || isnan(newPoint.z))
                continue;

            newPointcloud->push_back(newPoint);

            pcl::PointXYZ pclPoint;
            pclPoint.x = newPoint.x;
            pclPoint.y = newPoint.y;
            pclPoint.z = newPoint.z;
            pclCloud.push_back(pclPoint);
        }
    }

    pcl::io::savePLYFileASCII("output_cloud.ply", pclCloud); // Change the file name as needed

    ros::Rate rate(1.0); // Update once per second

    for (int i = 0; i < 1; i++) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*newPointcloud, cloud_msg);
        
        cloud_msg.header.frame_id = "nav_kinect_depth_optical_frame";

        cloud_msg.header.stamp = ros::Time::now();

        cloudPub.publish(cloud_msg);
        cout << "Published point cloud" << endl;
        rate.sleep();
    }
 
}

void SegmentationNetwork::processFile(const fs::path& filePath)
{
    std::cout << "New file added: " << filePath.string() << std::endl;

    std::string filename = filePath.filename();
    size_t dotPos = filename.find_last_of('.');
    filename = filename.substr(0, dotPos);
    int frame_idx = stoi(filename);
    cv::Mat label_mat = cv::imread(filePath.string(), cv::IMREAD_GRAYSCALE);
    projectPoints(frame_idx, label_mat);
}

void monitorFolder(const fs::path& folderPath, SegmentationNetwork *segmentation)
{
    std::set<std::string> processedFiles;

    std::cout << "Waiting for files added to " << folderPath.string() << std::endl;

    while (ros::ok())
    {
        for (const auto& entry : fs::directory_iterator(folderPath))
        {
            const fs::path& filePath = entry.path();
            if (fs::is_regular_file(filePath))
            {
                std::string filename = filePath.filename().string();
                if (processedFiles.find(filename) == processedFiles.end())
                {
                    segmentation->processFile(filePath);
                    processedFiles.insert(filename);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "semantic_cloud");
    ros::NodeHandle n;

    cloudPub = n.advertise<sensor_msgs::PointCloud2>("cloud_in", 1);

    SegmentationNetwork* segmentation = new SegmentationNetwork();
    std::vector<cv::Mat> label_mats;
    std::vector<int> frame_idxs;

    // std::string label_dir = dataDirectory + "depth";  // Base directory path
    std::string label_dir = dataDirectory + "labels";  // Base directory path

    std::cout << "Loading labels from " << label_dir << std::endl;

    monitorFolder(label_dir, segmentation);
}