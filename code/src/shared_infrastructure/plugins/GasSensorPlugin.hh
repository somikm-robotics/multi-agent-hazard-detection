#pragma once
#include <ignition/gazebo/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <shared_interfaces/msg/toxicity_result.hpp>

class GasSensorPlugin : public ignition::gazebo::System,
                        public ignition::gazebo::ISystemConfigure,
                        public ignition::gazebo::ISystemPreUpdate
{
    void Configure(const ignition::gazebo::Entity &,
                   const std::shared_ptr<const sdf::Element> &,
                   ignition::gazebo::EntityComponentManager &,
                   ignition::gazebo::EventManager &) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo &,
                   ignition::gazebo::EntityComponentManager &) override;

private:
    // ‑‑‑ SDF parameters  (must be supplied)
    std::string emitter_topic_;
    std::string sensor_topic_;
    std::string hazard_model_; // name of emitter model
    double decay_len_{3.0};    //  metres

    // ‑‑‑ ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<shared_interfaces::msg::ToxicityResult>::SharedPtr pub_;

    std_msgs::msg::Float32MultiArray raw_;
    bool have_raw_{false};

    // ‑‑‑ Entities for poses
    ignition::gazebo::Entity self_link_;
    ignition::gazebo::Entity hazard_ent_;
};
