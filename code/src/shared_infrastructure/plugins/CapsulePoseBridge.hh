#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h> // ignition.msgs.Pose_V

class CapsulePoseBridge : public rclcpp::Node
{
public:
    explicit CapsulePoseBridge(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

private:
    void onPoseV(const gz::msgs::Pose_V &msg);

    std::string capsule_name_;
    std::string world_topic_;
    std::string frame_id_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    gz::transport::Node gz_node_;
};
