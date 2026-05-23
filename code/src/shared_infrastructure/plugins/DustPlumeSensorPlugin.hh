#pragma once
#include <ignition/gazebo/System.hh> // System-plugin API
#include <rclcpp/rclcpp.hpp>
#include <shared_interfaces/msg/dust_sensor_result.hpp>

class DustPlumeSensorPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemPostUpdate
{
    /* --------- ISystemConfigure (called once, plume model entity) ---- */
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &,
                   ignition::gazebo::EventManager &) override;

    /* --------- Lazy ROS init + entity resolve ------------------------ */
    void PreUpdate(const ignition::gazebo::UpdateInfo &,
                   ignition::gazebo::EntityComponentManager &ecm) override;

    /* --------- Publish dust packet every step ------------------------ */
    void PostUpdate(const ignition::gazebo::UpdateInfo &,
                    const ignition::gazebo::EntityComponentManager &ecm) override;

    /* --------- helper ------------------------------------------------ */
    double InsideFactor(const ignition::math::Vector3d &p,
                        const ignition::math::Pose3d &plumePose) const;

    /* parameters */
    double radius_{5.0}, height_{3.0};
    double pm1Max_{220.0}, pm2Max_{550.0}, pm10Max_{900.0};
    double windSpeed_{3.8}, windDirDeg_{120.0},
        humidity_{38.0}, temperature_{28.0};
    std::string targetName_;

    /* cached entities */
    ignition::gazebo::Entity plumeEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity targetEntity_{ignition::gazebo::kNullEntity};

    /* ROS */
    rclcpp::Node::SharedPtr rosNode_{nullptr};
    rclcpp::Publisher<shared_interfaces::msg::DustSensorResult>::SharedPtr pub_;

    bool enabled_{true};
};
