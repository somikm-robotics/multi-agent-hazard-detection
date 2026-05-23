#pragma once
#include <ignition/gazebo/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <ignition/gazebo/components/Pose.hh>

class GasEmitterPlugin : public ignition::gazebo::System,
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
    // SDF parameters
    std::string topic_;  // must be provided in <topic>
    double rate_{10.0};  //  Hz
    double noise_{0.05}; //  ±fraction
    ignition::gazebo::Entity selfEnt_{ignition::gazebo::kNullEntity};

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;

    double last_pub_{0.0};

    // --- one‑off baseline values ------------------------------------------
    double co0_{0.0}, nh30_{0.0}, no20_{0.0}, voc0_{0.0};

    // helper to draw once from [lo,hi]
    static double Uniform(double lo, double hi)
    {
        return ignition::math::Rand::DblUniform(lo, hi);
    }
};
