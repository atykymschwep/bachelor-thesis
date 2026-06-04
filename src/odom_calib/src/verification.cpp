#include "odom_calib/robot_params.hpp"
#include "odom_calib/math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>
#include <string>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: ros2 run odom_calib verification <path_to_file.mcap> <new_R> <new_B>" << std::endl;
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

    RobotParams p_old; 
    RobotParams p_new; 
    
    try {
        p_new.R = std::stod(argv[2]);
        p_new.B = std::stod(argv[3]);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing R and B. Make sure they are numbers." << std::endl;
        return 1;
    }

    std::cout << "Starting verification with parameters:" << std::endl;
    std::cout << "Old R: " << p_old.R << " | Old B: " << p_old.B << std::endl;
    std::cout << "New R: " << p_new.R << " | New B: " << p_new.B << std::endl;

    long last_l = 0, last_r = 0;
    bool first_gt = false, first_enc = false;

    // point and angle start OptiTrack
    double gt_start_x = 0;
    double gt_start_y = 0;
    double gt_start_yaw = 0;
    double current_gt_x = 0;
    double current_gt_y = 0;

    // coord of odemeria
    double old_odom_x = 0;
    double old_odom_y = 0;
    double old_odom_yaw = 0;

    double new_odom_x = 0;
    double new_odom_y = 0;
    double new_odom_yaw = 0;

    std::ofstream csv_file("src/odom_calib/trajectory_data.csv");
    csv_file << "gt_x,gt_y,old_x,old_y,new_x,new_y\n";

    std::cout << "Exporting trajectory data..." << std::endl;

    while (reader.has_next()) {
        auto bag_message = reader.read_next();

        // read Ground Truth (Optitrack)
        if (bag_message->topic_name == "/Kobuki/pose") {
            auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            auto serializer = rclcpp::Serialization<geometry_msgs::msg::PoseStamped>();
            serializer.deserialize_message(&serialized_msg, msg.get());

            current_gt_x = msg->pose.position.x;
            current_gt_y = msg->pose.position.y;

            // calc current YAW from Q
            double qx = msg->pose.orientation.x;
            double qy = msg->pose.orientation.y;
            double qz = msg->pose.orientation.z;
            double qw = msg->pose.orientation.w;
            double current_gt_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

            if (!first_gt) {
                gt_start_x = current_gt_x;
                gt_start_y = current_gt_y;
                gt_start_yaw = current_gt_yaw;
                first_gt = true;
            }
        }

        // read encoders and calc odomeria
        if (bag_message->topic_name == "/kobuki/sensors_raw" && first_gt) {
            auto msg = std_msgs::msg::Int32MultiArray();
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            auto serializer = rclcpp::Serialization<std_msgs::msg::Int32MultiArray>();
            serializer.deserialize_message(&serialized_msg, &msg);

            if (msg.data.size() < 2) continue;
            
            if (!first_enc) {
                last_l = msg.data[0]; 
                last_r = msg.data[1];
                first_enc = true; 
                continue;
            }

            // calc old odom
            long diff_l_old = correct_overflow(msg.data[0] - last_l, p_old);
            long diff_r_old = correct_overflow(msg.data[1] - last_r, p_old);
            double ds_old = (ticks_to_meters(diff_l_old, p_old) + ticks_to_meters(diff_r_old, p_old)) / 2.0;
            double dth_old = (ticks_to_meters(diff_r_old, p_old) - ticks_to_meters(diff_l_old, p_old)) / p_old.B;

            old_odom_x += ds_old * std::cos(old_odom_yaw + dth_old / 2.0);
            old_odom_y += ds_old * std::sin(old_odom_yaw + dth_old / 2.0);
            old_odom_yaw = normalize_angle(old_odom_yaw + dth_old);

            // calc new odom
            long diff_l_new = correct_overflow(msg.data[0] - last_l, p_new);
            long diff_r_new = correct_overflow(msg.data[1] - last_r, p_new);
            double ds_new = (ticks_to_meters(diff_l_new, p_new) + ticks_to_meters(diff_r_new, p_new)) / 2.0;
            double dth_new = (ticks_to_meters(diff_r_new, p_new) - ticks_to_meters(diff_l_new, p_new)) / p_new.B;

            new_odom_x += ds_new * std::cos(new_odom_yaw + dth_new / 2.0);
            new_odom_y += ds_new * std::sin(new_odom_yaw + dth_new / 2.0);
            new_odom_yaw = normalize_angle(new_odom_yaw + dth_new);

            // align axes of OptiTrack
            double dx = current_gt_x - gt_start_x;
            double dy = current_gt_y - gt_start_y;

            // rotate on -gt_start_yaw to OptiTrack start with 0 rad
            double gt_x_aligned = dx * std::cos(-gt_start_yaw) - dy * std::sin(-gt_start_yaw);
            double gt_y_aligned = dx * std::sin(-gt_start_yaw) + dy * std::cos(-gt_start_yaw);

            // read in file OptiTrack
            csv_file << gt_x_aligned << "," 
                     << gt_y_aligned << ","
                     << old_odom_x << "," << old_odom_y << ","
                     << new_odom_x << "," << new_odom_y << "\n";

            last_l = msg.data[0]; 
            last_r = msg.data[1];
        }
    }

    csv_file.close();
    std::cout << "Done! Saved to trajectory_data.csv" << std::endl;
    return 0;
}