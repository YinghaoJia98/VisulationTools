#include <VisualTools/VisualTools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizationTools");
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    using namespace VT;
    ReadParameters(nh_);

    ros::Subscriber subOdometry = nh_.subscribe<nav_msgs::Odometry>(OdoTopic_.c_str(), 5, VT::odometryHandler);
    ros::Subscriber subLaserCloud = nh_.subscribe<sensor_msgs::PointCloud2>(LaserTopic_.c_str(), 5, VT::laserCloudHandler);
    ros::Subscriber subRuntime = nh_.subscribe<std_msgs::Float32>(TimeTopic_.c_str(), 5, VT::runtimeHandler);

    ros::Publisher pubOverallMap = nh_.advertise<sensor_msgs::PointCloud2>("/overall_map", 5);

    ros::Publisher pubExploredArea = nh_.advertise<sensor_msgs::PointCloud2>("/explored_areas", 5);
    pubExploredAreaPtr = &pubExploredArea;

    ros::Publisher pubTrajectory = nh_.advertise<sensor_msgs::PointCloud2>("/trajectory", 5);
    pubTrajectoryPtr = &pubTrajectory;

    ros::Publisher pubExploredVolume = nh_.advertise<std_msgs::Float32>("/explored_volume", 5);
    pubExploredVolumePtr = &pubExploredVolume;

    ros::Publisher pubTravelingDis = nh_.advertise<std_msgs::Float32>("/traveling_distance", 5);
    pubTravelingDisPtr = &pubTravelingDis;

    ros::Publisher pubTimeDuration = nh_.advertise<std_msgs::Float32>("/time_duration", 5);
    pubTimeDurationPtr = &pubTimeDuration;

    // ros::Publisher pubRuntime = nh_.advertise<std_msgs::Float32> ("/runtime", 5);

    overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
    exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
    exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

    pcl::PLYReader ply_reader;
    if (ply_reader.read(mapFile, *overallMapCloud) == -1)
    {
        printf("\nCouldn't read pointcloud.ply file.\n\n");
    }

    overallMapCloudDwz->clear();
    overallMapDwzFilter.setInputCloud(overallMapCloud);
    overallMapDwzFilter.filter(*overallMapCloudDwz);
    overallMapCloud->clear();

    pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

    time_t logTime = time(0);
    tm *ltm = localtime(&logTime);
    std::string timeString = std::to_string(1900 + ltm->tm_year) + "-" + std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) + "-" +
                             std::to_string(ltm->tm_hour) + "-" + std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec);

    metricFile += "_" + timeString + ".txt";
    trajFile += "_" + timeString + ".txt";
    metricFilePtr = fopen(metricFile.c_str(), "w");
    trajFilePtr = fopen(trajFile.c_str(), "w");

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();

        overallMapDisplayCount++;
        if (overallMapDisplayCount >= 100 * overallMapDisplayInterval)
        {
            overallMap2.header.stamp = ros::Time().fromSec(systemTime);
            overallMap2.header.frame_id = "map";
            pubOverallMap.publish(overallMap2);

            overallMapDisplayCount = 0;
        }

        status = ros::ok();
        rate.sleep();
    }

    fclose(metricFilePtr);
    fclose(trajFilePtr);

    printf("\nExploration metrics and vehicle trajectory are saved.\n\n");

    return 0;
}