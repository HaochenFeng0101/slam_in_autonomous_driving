// //
// // Created by xiang on 2022/7/18.
// //

// #include <gflags/gflags.h>
// #include <glog/logging.h>

// #include "ch7/loam-like/loam_like_odom.h"
// #include "common/io_utils.h"
// #include "common/timer/timer.h"

// DEFINE_string(bag_path, "./dataset/sad/wxb/test1.bag", "path to wxb bag");
// DEFINE_string(topic, "/velodyne_packets_1", "topic of lidar packets");
// DEFINE_bool(display_map, true, "display map?");

// int main(int argc, char** argv) {
//     google::InitGoogleLogging(argv[0]);
//     FLAGS_stderrthreshold = google::INFO;
//     FLAGS_colorlogtostderr = true;
//     google::ParseCommandLineFlags(&argc, &argv, true);

//     // 测试loam-like odometry的表现
//     sad::LoamLikeOdom::Options options;
//     options.display_realtime_cloud_ = FLAGS_display_map;
//     sad::LoamLikeOdom lo(options);

//     LOG(INFO) << "using topic: " << FLAGS_topic;
//     sad::RosbagIO bag_io(fLS::FLAGS_bag_path);
//     bag_io
//         .AddVelodyneHandle(FLAGS_topic,
//                            [&](sad::FullCloudPtr cloud) -> bool {
//                                sad::common::Timer::Evaluate([&]() { lo.ProcessPointCloud(cloud); }, "Loam-like
//                                odom"); return true;
//                            })
//         .Go();

//     lo.SaveMap("./data/ch7/loam_map.pcd");

//     sad::common::Timer::PrintAll();
//     LOG(INFO) << "done.";

//     return 0;
// }
//
// Created by xiang on 2022/7/18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch7/loam-like/loam_like_odom.h"
#include "common/io_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "/home/rayz2004/code/slam_in_autonomous_driving/dataset/wxb/road.bag", "path to wxb bag");
DEFINE_string(topic, "/publisher/rayz_points", "topic of lidar packets");
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 测试loam-like odometry的表现
    sad::LoamLikeOdom::Options options;
    options.display_realtime_cloud_ = FLAGS_display_map;
    sad::LoamLikeOdom lo(options);

    LOG(INFO) << "using topic: " << FLAGS_topic;
    sad::RosbagIO bag_io(fLS::FLAGS_bag_path);
    // bag_io
    //     .AddVelodyneHandle(FLAGS_topic,
    //                        [&](sad::FullCloudPtr cloud) -> bool {
    //                            sad::common::Timer::Evaluate([&]() { lo.ProcessPointCloud(cloud); }, "Loam-like
    //                            odom"); return true;
    //                        })
    //     .Go();
    bag_io
        .AddPointCloud2Handle(FLAGS_topic,
                              [&](boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg) -> bool {
                                  // Create a new FullCloudPtr
                                  sad::FullCloudPtr cloud(new sad::FullPointCloudType());

                                  // Convert from ROS PointCloud2 to PCL point cloud
                                  pcl::fromROSMsg(*cloud_msg, *cloud);

                                  for (const auto& field : cloud_msg->fields) {
                                      ROS_INFO("    - Name: %s, Offset: %d, Datatype: %d, Count: %d",
                                               field.name.c_str(), field.offset, field.datatype, field.count);
                                  }

                                  // Process the converted point cloud
                                  sad::common::Timer::Evaluate([&]() { lo.ProcessPointCloud(cloud); },
                                                               "Loam-like odom");
                                  return true;
                              })
        .Go();

    lo.SaveMap("./data/ch7/loam_map.pcd");

    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
