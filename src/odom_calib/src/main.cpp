#include "odom_calib/robot_params.hpp"
#include "odom_calib/math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <iostream>
#include <memory>
#include <cmath>
#include <iomanip>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Using: ros2 run odom_calib odom_calib <path_to_file.mcap>" << std::endl;
        return 1;
    }

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = argv[1];
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    RobotParams p;
    long last_l = 0;
    long last_r = 0;
    bool first_gt = false;
    bool first_enc = false;

    double current_gt_x = 0; 
    double current_gt_y = 0; 
    double current_gt_yaw = 0;

    double last_sync_gt_x = 0; 
    double last_sync_gt_y = 0; 
    double last_sync_gt_yaw = 0;

    // accum variables
    double total_dist_odom = 0;
    double total_dist_gt = 0;
    double total_th_odom = 0;
    double total_th_gt = 0;

    // variabs for cumulative least squares
    double sum_X2_s = 0; 
    double sum_XY_s = 0;
    double sum_X2_th = 0;
    double sum_XY_th = 0;

    int msg_count = 0;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "- Trajectory -" << std::endl;

    while (reader.has_next()) {
        auto bag_message = reader.read_next();

        // OptiTrack
        if (bag_message->topic_name == "/Kobuki/pose") {
            auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            auto serializer = rclcpp::Serialization<geometry_msgs::msg::PoseStamped>();
            serializer.deserialize_message(&serialized_msg, msg.get());

            current_gt_x = msg->pose.position.x;
            current_gt_y = msg->pose.position.y;
            current_gt_yaw = quaternion_to_yaw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

            if (!first_gt) {
                last_sync_gt_x = current_gt_x; 
                last_sync_gt_y = current_gt_y;
                last_sync_gt_yaw = current_gt_yaw;
                first_gt = true;
            }
        }

        // odometry
        if (bag_message->topic_name == "/kobuki/sensors_raw" && first_gt) {
            auto msg = std_msgs::msg::Int32MultiArray();
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            auto serializer = rclcpp::Serialization<std_msgs::msg::Int32MultiArray>();
            serializer.deserialize_message(&serialized_msg, &msg);

            if (msg.data.size() < 2) continue;
            if (!first_enc) {
                last_l = msg.data[0]; last_r = msg.data[1];
                first_enc = true; continue;
            }
            
            // delts
            long diff_l = correct_overflow(msg.data[0] - last_l, p);
            long diff_r = correct_overflow(msg.data[1] - last_r, p);

            double sl = ticks_to_meters(diff_l, p);
            double sr = ticks_to_meters(diff_r, p);
            
            double ds_odom = (sl + sr) / 2.0;
            double dth_odom = (sr - sl) / p.B;

            double d_gt = calculate_distance(last_sync_gt_x, last_sync_gt_y, current_gt_x, current_gt_y);
            double dth_gt = normalize_angle(current_gt_yaw - last_sync_gt_yaw);

            // update dist
            total_dist_odom += std::abs(ds_odom);
            total_dist_gt += d_gt;
            
            total_th_odom += std::abs(dth_odom);
            total_th_gt += std::abs(dth_gt);

            // formula: sum(X^2) and sum(X*Y), where X = odom, Y = gt
            sum_X2_s += total_dist_odom * total_dist_odom;
            sum_XY_s += total_dist_odom * total_dist_gt;

            sum_X2_th += total_th_odom * total_th_odom;
            sum_XY_th += total_th_odom * total_th_gt;

            // save previous values
            last_l = msg.data[0]; 
            last_r = msg.data[1];
            last_sync_gt_x = current_gt_x; 
            last_sync_gt_y = current_gt_y;
            last_sync_gt_yaw = current_gt_yaw;

            if (++msg_count % 50 == 0) {
                std::cout << "DIST | GT: " << total_dist_gt << " m | Odom: " << total_dist_odom 
                          << " m | Err: " << (total_dist_gt - total_dist_odom) << " m" << std::endl;
            }
        }
    }

    std::cout << "- finished -\n" << std::endl;

    // division zero
    if (sum_X2_s > 0.0001 && sum_X2_th > 0.0001) {
          
        // calc koef 
        double k_s = sum_XY_s / sum_X2_s;
        double k_th = sum_XY_th / sum_X2_th;

        std::cout << " - Results -" << std::endl;
        std::cout << " linial correction (k-s): " << k_s << std::endl;
        std::cout << " angular correction (k-th): " << k_th << std::endl;
        std::cout << "---" << std::endl;
        
        std::cout << " new R: " << (p.R * k_s) << " m" << std::endl;
        std::cout << " new B: " << (p.B / k_th) << " m" << std::endl;
        std::cout << "---" << std::endl;
        
    } else {
        std::cout << "Error: Not enough movement data for calibration!" << std::endl;
    }

    return 0;
}