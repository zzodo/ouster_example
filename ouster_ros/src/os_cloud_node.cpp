/**
 * @file
 * @brief Example node to publish point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using Cloud = ouster_ros::Cloud;
using Point = ouster_ros::Point;
namespace sensor = ouster::sensor;

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_cloud_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle pnh("");

    auto tf_prefix = nh.param("tf_prefix", std::string{});
    if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
    auto sensor_frame = tf_prefix + "os_sensor";
    auto imu_frame = tf_prefix + "os_imu";
    auto lidar_frame = tf_prefix + "os_lidar";
    auto packet_stride = nh.param<int>("packet_stride", 16);

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
//    auto client = pnh.serviceClient<ouster_ros::OSConfigSrv>("/os_node/os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);

    auto pf_old = sensor::get_format(info);
    const int divisor = info.format.columns_per_frame / (pf_old.columns_per_packet * packet_stride);
    info.format.columns_per_frame = info.format.columns_per_frame / divisor;
    auto pf = sensor::get_format(info);
    auto udp_profile_lidar = info.format.udp_profile_lidar;

    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto img_suffix = [](int ind) {
        if (ind == 0) return std::string();
        return std::to_string(ind + 1);  // need second return to return 2
    };

    auto lidar_pubs = std::vector<ros::Publisher>();
    for (int i = 0; i < divisor; i++) {
        auto pub = nh.advertise<sensor_msgs::PointCloud2>(
                std::string("points") + img_suffix(i), 10);
        lidar_pubs.push_back(pub);
    }

    uint32_t H = info.format.pixels_per_column;
    uint32_t W = info.format.columns_per_frame;

    Eigen::Matrix<double, 4, 4, 2> rot_division = Eigen::Matrix<double, 4, 4, 2>::Identity();
    double rot_angle_radians = 2 * M_PI / divisor;
    rot_division (0, 0) = std::cos(rot_angle_radians);
    rot_division (0, 1) = -std::sin(rot_angle_radians);
    rot_division (1, 0) = std::sin(rot_angle_radians);
    rot_division (1, 1) = std::cos(rot_angle_radians);

    auto xyz_luts = std::vector<ouster::XYZLut>();
    for (int i = 0; i < divisor; i++) {
        Eigen::Matrix<double, 4, 4, 2> transformation = info.lidar_to_sensor_transform;
        for (int j = 0; j < i; j++)
            transformation = transformation * rot_division;
        auto xyz_lut = ouster::make_xyz_lut(W, H,
                                            sensor::range_unit, info.lidar_origin_to_beam_origin_mm,
                                            transformation, info.beam_azimuth_angles,
                                            info.beam_altitude_angles, divisor);
        xyz_luts.push_back(xyz_lut);
    }

    ouster::LidarScan ls{W, H, udp_profile_lidar};
    Cloud cloud{W, H};

    size_t division_id = 0;
    ouster::ScanBatcher batch(W, pf);

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        if (batch(pm.buf.data(), ls)) {
            auto h = std::min_element(
                    ls.headers.begin(), ls.headers.end(), [](const auto &h1, const auto &h2) {
                        return h1.timestamp < h2.timestamp;
                    });
            scan_to_cloud(xyz_luts[division_id % divisor], h->timestamp, ls, cloud, 0);
            lidar_pubs[division_id % divisor].publish(ouster_ros::cloud_to_cloud_msg(
                    cloud, h->timestamp, sensor_frame));
            division_id++;
        }
    };

    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::packet_to_imu_msg(p, imu_frame, pf));
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);
//    auto lidar_packet_sub = pnh.subscribe<PacketMsg, const PacketMsg&>(
//            "/os_node/lidar_packets", 2048, lidar_handler);
//    auto imu_packet_sub = pnh.subscribe<PacketMsg, const PacketMsg&>(
//            "/os_node/imu_packets", 100, imu_handler);

    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}
