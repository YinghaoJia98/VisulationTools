#ifndef VT_H_
#define VT_H_

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace VT
{
    const double PI = 3.1415926;
    std::string metricFile;
    std::string trajFile;
    std::string mapFile;
    double overallMapVoxelSize;
    double exploredAreaVoxelSize;
    double exploredVolumeVoxelSize;
    double transInterval;
    double yawInterval;
    int overallMapDisplayInterval;
    int overallMapDisplayCount;
    int exploredAreaDisplayInterval;
    int exploredAreaDisplayCount;

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

    const int systemDelay = 5;
    int systemDelayCount = 0;
    bool systemDelayInited = false;
    double systemTime = 0;
    double systemInitTime = 0;
    bool systemInited = false;

    float vehicleYaw = 0;
    float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
    float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;

    pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
    pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
    pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

    sensor_msgs::PointCloud2 overallMap2;

    ros::Publisher *pubExploredAreaPtr = NULL;
    ros::Publisher *pubTrajectoryPtr = NULL;
    ros::Publisher *pubExploredVolumePtr = NULL;
    ros::Publisher *pubTravelingDisPtr = NULL;
    ros::Publisher *pubTimeDurationPtr = NULL;

    FILE *metricFilePtr = NULL;
    FILE *trajFilePtr = NULL;

    std::string OdoTopic_;
    std::string LaserTopic_;
    std::string TimeTopic_;

    void ReadParameters(ros::NodeHandle nh);

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom);
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn);
    void runtimeHandler(const std_msgs::Float32::ConstPtr &runtimeIn);

    void ReadParameters(ros::NodeHandle nh)
    {
        nh.param<std::string>("/VisualizationTools/VTParameters/metricFile",
                              metricFile, std::string("../log/metrics"));
        nh.param<std::string>("/VisualizationTools/VTParameters/trajFile",
                              trajFile, std::string("../log/trajFile"));
        nh.param<std::string>("/VisualizationTools/VTParameters/mapFile",
                              mapFile, std::string("../mesh/map.ply"));
        nh.param<double>("/VisualizationTools/VTParameters/overallMapVoxelSize",
                         overallMapVoxelSize, 0.5);
        nh.param<double>("/VisualizationTools/VTParameters/exploredAreaVoxelSize",
                         exploredAreaVoxelSize, 0.3);
        nh.param<double>("/VisualizationTools/VTParameters/exploredVolumeVoxelSize",
                         exploredVolumeVoxelSize, 0.5);
        nh.param<double>("/VisualizationTools/VTParameters/transInterval",
                         transInterval, 0.2);
        nh.param<double>("/VisualizationTools/VTParameters/yawInterval",
                         yawInterval, 10.0);
        nh.param<int>("/VisualizationTools/VTParameters/overallMapDisplayInterval",
                      overallMapDisplayInterval, 2);
        nh.param<int>("/VisualizationTools/VTParameters/exploredAreaDisplayInterval",
                      exploredAreaDisplayInterval, 1);

        nh.param<std::string>("/VisualizationTools/VTParameters/OdoTopic",
                              OdoTopic_, std::string("/state_estimation"));
        nh.param<std::string>("/VisualizationTools/VTParameters/LaserTopic",
                              LaserTopic_, std::string("/registered_scan"));
        nh.param<std::string>("/VisualizationTools/VTParameters/TimeTopic",
                              TimeTopic_, std::string("/runtime"));
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
    {
        systemTime = odom->header.stamp.toSec();

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

        float dYaw = fabs(yaw - vehicleYaw);
        if (dYaw > PI)
            dYaw = 2 * PI - dYaw;

        float dx = odom->pose.pose.position.x - vehicleX;
        float dy = odom->pose.pose.position.y - vehicleY;
        float dz = odom->pose.pose.position.z - vehicleZ;
        float dis = sqrt(dx * dx + dy * dy + dz * dz);

        if (!systemDelayInited)
        {
            vehicleYaw = yaw;
            vehicleX = odom->pose.pose.position.x;
            vehicleY = odom->pose.pose.position.y;
            vehicleZ = odom->pose.pose.position.z;
            return;
        }

        if (systemInited)
        {
            timeDuration = systemTime - systemInitTime;

            std_msgs::Float32 timeDurationMsg;
            timeDurationMsg.data = timeDuration;
            pubTimeDurationPtr->publish(timeDurationMsg);
        }

        if (dis < transInterval && dYaw < yawInterval)
        {
            return;
        }

        if (!systemInited)
        {
            dis = 0;
            systemInitTime = systemTime;
            systemInited = true;
        }

        travelingDis += dis;

        vehicleYaw = yaw;
        vehicleX = odom->pose.pose.position.x;
        vehicleY = odom->pose.pose.position.y;
        vehicleZ = odom->pose.pose.position.z;

        fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);

        pcl::PointXYZI point;
        point.x = vehicleX;
        point.y = vehicleY;
        point.z = vehicleZ;
        point.intensity = travelingDis;
        trajectory->push_back(point);

        sensor_msgs::PointCloud2 trajectory2;
        pcl::toROSMsg(*trajectory, trajectory2);
        trajectory2.header.stamp = odom->header.stamp;
        trajectory2.header.frame_id = "map";
        pubTrajectoryPtr->publish(trajectory2);
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn)
    {
        if (!systemDelayInited)
        {
            systemDelayCount++;
            if (systemDelayCount > systemDelay)
            {
                systemDelayInited = true;
            }
        }

        if (!systemInited)
        {
            return;
        }

        laserCloud->clear();
        pcl::fromROSMsg(*laserCloudIn, *laserCloud);

        *exploredVolumeCloud += *laserCloud;

        exploredVolumeCloud2->clear();
        exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
        exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
        exploredVolumeCloud = exploredVolumeCloud2;
        exploredVolumeCloud2 = tempCloud;

        exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize *
                         exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

        *exploredAreaCloud += *laserCloud;

        exploredAreaDisplayCount++;
        if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval)
        {
            exploredAreaCloud2->clear();
            exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
            exploredAreaDwzFilter.filter(*exploredAreaCloud2);

            tempCloud = exploredAreaCloud;
            exploredAreaCloud = exploredAreaCloud2;
            exploredAreaCloud2 = tempCloud;

            sensor_msgs::PointCloud2 exploredArea2;
            pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
            exploredArea2.header.stamp = laserCloudIn->header.stamp;
            exploredArea2.header.frame_id = "map";
            pubExploredAreaPtr->publish(exploredArea2);

            exploredAreaDisplayCount = 0;
        }

        fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

        std_msgs::Float32 exploredVolumeMsg;
        exploredVolumeMsg.data = exploredVolume;
        pubExploredVolumePtr->publish(exploredVolumeMsg);

        std_msgs::Float32 travelingDisMsg;
        travelingDisMsg.data = travelingDis;
        pubTravelingDisPtr->publish(travelingDisMsg);
    }
    void runtimeHandler(const std_msgs::Float32::ConstPtr &runtimeIn)
    {
        runtime = runtimeIn->data;
    }

}

#endif /* VT_H_ */
