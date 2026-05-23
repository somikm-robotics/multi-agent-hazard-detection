#include "DustPlumeSensorPlugin.hh"

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/math/Helpers.hh>
#include <rclcpp/utilities.hpp>
#include <thread>

namespace ig = ignition::gazebo;

/* ---------------- Configure (runs once) --------------------------- */
void DustPlumeSensorPlugin::Configure(const ig::Entity &_entity,
                                      const std::shared_ptr<const sdf::Element> &_sdf,
                                      ig::EntityComponentManager &,
                                      ig::EventManager &)
{
    plumeEntity_ = _entity; // this plume model

    if (!_sdf->HasElement("target_model"))
    {
        ignerr << "[DustPlumeSensorPlugin] <target_model> tag is required\n";
        enabled_ = false;
        return;
    }
    targetName_ = _sdf->Get<std::string>("target_model");

    _sdf->Get<double>("radius", radius_, radius_);
    _sdf->Get<double>("height", height_, height_);
    _sdf->Get<double>("pm1_0_max", pm1Max_, pm1Max_);
    _sdf->Get<double>("pm2_5_max", pm2Max_, pm2Max_);
    _sdf->Get<double>("pm10_max", pm10Max_, pm10Max_);
    _sdf->Get<double>("wind_speed", windSpeed_, windSpeed_);
    _sdf->Get<double>("wind_direction", windDirDeg_, windDirDeg_);
    _sdf->Get<double>("humidity", humidity_, humidity_);
    _sdf->Get<double>("temperature", temperature_, temperature_);
}

/* ---------------- PreUpdate – GasSensor pattern ------------------- */
void DustPlumeSensorPlugin::PreUpdate(const ig::UpdateInfo &,
                                      ig::EntityComponentManager &ecm)
{
    /* ─── early-out if the plugin was disabled in Configure() ─── */
    if (!this->enabled_)
        return;

    /* ─── 1. Lazy ROS node + publisher initialisation (GasSensor style) ─── */
    if (!this->rosNode_)
    {
        if (!rclcpp::ok()) // ensure single ROS context
        {
            int argc = 1;
            const char *argv[] = {"dust_plume_sensor"};
            try
            {
                rclcpp::init(argc, const_cast<char **>(argv));
            }
            catch (const std::runtime_error &e)
            {
                ignerr << "[DustPlumeSensorPlugin] ROS init failed: "
                       << e.what() << std::endl;
                throw; // abort world load – no silent failure
            }
        }

        this->rosNode_ = rclcpp::Node::make_shared("dust_plume_sensor");
        this->pub_ = this->rosNode_->create_publisher<
            shared_interfaces::msg::DustSensorResult>(
            "/dust/" + this->targetName_, 10);

        ignmsg << "[DustPlumeSensorPlugin] ROS node initialised; "
               << "publishing on /dust/" << this->targetName_ << std::endl;
    }

    /* ─── 2. Allow any ROS callbacks to run this sim tick ─── */
    rclcpp::spin_some(this->rosNode_);

    /* ─── 3. Resolve the robot entity once ─── */
    if (this->targetEntity_ == ig::kNullEntity)
    {
        ecm.Each<ig::components::Name>(
            [&](const ig::Entity &ent,
                const ig::components::Name *name) -> bool
            {
                if (name && name->Data() == this->targetName_)
                {
                    this->targetEntity_ = ent;
                    return false; // found → stop iterating
                }
                return true; // keep searching
            });
    }
}

/* ---------------- helper ------------------------------------------ */
double DustPlumeSensorPlugin::InsideFactor(const ignition::math::Vector3d &p,
                                           const ignition::math::Pose3d &pl) const
{
    auto d = p - pl.Pos();
    double r = std::hypot(d.X(), d.Y());
    double z = std::fabs(d.Z());
    if (r >= radius_ || z >= height_ / 2.0)
        return 0.0;
    return 1.0 - (r / radius_);
}

/* ---------------- PostUpdate – publish dust ----------------------- */
void DustPlumeSensorPlugin::PostUpdate(const ig::UpdateInfo &,
                                       const ig::EntityComponentManager &ecm)
{
    if (!enabled_ || !rosNode_ || targetEntity_ == ig::kNullEntity)
        return;

    auto plumePose = ecm.Component<ig::components::Pose>(plumeEntity_);
    auto robotPose = ecm.Component<ig::components::Pose>(targetEntity_);
    if (!plumePose || !robotPose)
        return;

    double k = InsideFactor(robotPose->Data().Pos(), plumePose->Data());

    shared_interfaces::msg::DustSensorResult msg;
    msg.pm1_0 = k * pm1Max_;
    msg.pm2_5 = k * pm2Max_;
    msg.pm10 = k * pm10Max_;
    msg.tsp = msg.pm10;
    msg.opacity = k * (1.0 - std::exp(-0.003 * pm2Max_));
    msg.wind_speed = windSpeed_;
    msg.wind_direction = windDirDeg_;
    msg.humidity = humidity_;
    msg.temperature = temperature_;

    pub_->publish(msg);
}

/* ---------------- Register ---------------------------- */
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    DustPlumeSensorPlugin,
    ig::System,
    DustPlumeSensorPlugin::ISystemConfigure,
    DustPlumeSensorPlugin::ISystemPreUpdate,
    DustPlumeSensorPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DustPlumeSensorPlugin, "dust_plume_sensor")
