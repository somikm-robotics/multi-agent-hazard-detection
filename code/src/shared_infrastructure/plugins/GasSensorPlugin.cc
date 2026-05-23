#include "GasSensorPlugin.hh"
#include <ignition/plugin/Register.hh>
#include <rclcpp/utilities.hpp>
#include <cmath>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh> // ← brings in components::Model

using namespace ignition;
using namespace gazebo;

// --- local enum + helpers ---------------------------------------------
enum class Level
{
    ACCEPT,
    ELEV,
    DANGER,
    EMERG
};

static Level clsCO(double v) { return v <= 9 ? Level::ACCEPT : v <= 35 ? Level::ELEV
                                                           : v <= 200  ? Level::DANGER
                                                                       : Level::EMERG; }
static Level clsNO2(double v) { return v <= .10 ? Level::ACCEPT : v <= 1 ? Level::ELEV
                                                              : v <= 5   ? Level::DANGER
                                                                         : Level::EMERG; }
static Level clsNH3(double v) { return v <= 5 ? Level::ACCEPT : v <= 25 ? Level::ELEV
                                                            : v <= 50   ? Level::DANGER
                                                                        : Level::EMERG; }
static Level clsVOC(double v) { return v <= 50 ? Level::ACCEPT : v <= 100 ? Level::ELEV
                                                             : v <= 200   ? Level::DANGER
                                                                          : Level::EMERG; }

static std::string toStr(Level L)
{
    switch (L)
    {
    case Level::EMERG:
        return "Emergency";
    case Level::DANGER:
        return "Dangerous";
    case Level::ELEV:
        return "Elevated";
    default:
        return "Acceptable";
    }
}

// ----------------------------------------------------------------------
void GasSensorPlugin::Configure(const Entity &entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager &ecm,
                                EventManager &)
{
    Model mdl(entity);
    for (auto l : mdl.Links(ecm))
    { // take first link pose
        self_link_ = l;
        break;
    }

    // ---- mandatory SDF parameters ---------------------------------------
    if (_sdf->HasElement("emitter_topic"))
        emitter_topic_ = _sdf->Get<std::string>("emitter_topic");
    else
        throw std::runtime_error(
            "[GasSensorPlugin] <emitter_topic> is required");

    if (_sdf->HasElement("sensor_topic"))
        sensor_topic_ = _sdf->Get<std::string>("sensor_topic");
    else
        throw std::runtime_error(
            "[GasSensorPlugin] <sensor_topic> is required");

    if (_sdf->HasElement("decay_length"))
        decay_len_ = _sdf->Get<double>("decay_length");

    if (_sdf->HasElement("hazard_model"))
        hazard_model_ = _sdf->Get<std::string>("hazard_model");

    for (auto e : ecm.EntitiesByComponents(components::Name(hazard_model_), components::Model()))
    {
        hazard_ent_ = e;
        break;
    }

    // ───── defer ROS node creation until ROS is already initialised ─────
    //   try{rclcpp::init(0,nullptr);}catch(...){}
    //   node_=rclcpp::Node::make_shared("gas_sensor");

    // sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    //     emitter_topic_, 10,
    //     [this](auto msg)
    //     {raw_=*msg; have_raw_=true; });

    //   pub_=node_->create_publisher<shared_interfaces::msg::ToxicityResult>(
    //       sensor_topic_,10);

    //   std::thread([n=node_]{rclcpp::spin(n);}).detach();
}

// ----------------------------------------------------------------------
void GasSensorPlugin::PreUpdate(const UpdateInfo &,
                                EntityComponentManager &ecm)
{
    // ───── 1. Lazy ROS node + sub/pub initialisation ──────────────────
    if (!this->node_)
    {
        // Ensure one and only one ROS context exists in this process
        if (!rclcpp::ok()) // ← use ok() instead of is_initialized()
        {
            int argc = 1;
            const char *argv[] = {"gas_sensor"};
            try
            {
                rclcpp::init(argc, const_cast<char **>(argv));
            }
            catch (const std::runtime_error &e)
            {
                ignerr << "[GasSensorPlugin] ROS init failed: "
                       << e.what() << std::endl;
                throw; // abort world load – no silent failure
            }
        }

        // Create node, subscriber, publisher exactly once
        // now ROS is up – create our node
        this->node_ = rclcpp::Node::make_shared("gas_sensor");

        // subscribe to the raw emitter
        this->sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            emitter_topic_, 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {raw_=*msg; have_raw_=true; });

        // publisher for scaled + status
        this->pub_ = node_->create_publisher<shared_interfaces::msg::ToxicityResult>(
            sensor_topic_, 10);

        ignmsg << "[GasSensorPlugin] ROS node initialised; "
               << "subscribed to " << emitter_topic_
               << " and publishing to " << sensor_topic_ << std::endl;
    }

    // 2.  allow subscriber callbacks to execute
    rclcpp::spin_some(this->node_);

    // need a message (6 floats) + our own link entity
    if (!have_raw_ || raw_.data.size() < 7 || !self_link_)
        return;

    // ───── self‑pose (World) ----------------------------------------------------
    auto selfPose = ecm.Component<components::WorldPose>(self_link_);
    if (!selfPose) // not yet available this tick
        return;

    // ───── hazard position comes from the message (index 4,5) ------------------
    double hx = static_cast<double>(raw_.data[4]);
    double hy = static_cast<double>(raw_.data[5]);

    const auto &selfPos = selfPose->Data().Pos();
    // const auto &hazPos = hazPos->Data().Pos();
    double dx = selfPos.X() - raw_.data[4]; // hazard x
    double dy = selfPos.Y() - raw_.data[5]; // hazard y
    double dz = selfPos.Z() - raw_.data[6]; // hazard z

    // double dist = selfPos.Distance(*hazPos); // √(dx²+dy²+dz²)
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz); // full 3‑D distance
    double k = std::exp(-dist / decay_len_);              // e^{‑d/L}

    // auto pSelf = *ecm.Component<components::Pose>(self_link_);
    // auto pHaz = *ecm.Component<components::Pose>(hazard_ent_);
    // double dx = pSelf.Data().Pos().X() - pHaz.Data().Pos().X();
    // double dy = pSelf.Data().Pos().Y() - pHaz.Data().Pos().Y();
    // double dist = std::hypot(dx, dy);
    // double k = std::exp(-dist / decay_len_);

    shared_interfaces::msg::ToxicityResult out;
    out.co_ppm = raw_.data[0] * k;
    out.nh3_ppm = raw_.data[1] * k;
    out.no2_ppm = raw_.data[2] * k;
    out.voc_ppb = raw_.data[3] * k;

    int lvlCO = static_cast<int>(clsCO(out.co_ppm));
    int lvlNO2 = static_cast<int>(clsNO2(out.no2_ppm));
    int lvlNH3 = static_cast<int>(clsNH3(out.nh3_ppm));
    int lvlVOC = static_cast<int>(clsVOC(out.voc_ppb));

    if (lvlCO >= 2 || lvlNO2 >= 2 || lvlNH3 >= 2 || lvlVOC >= 2)
    {
        Level worst = Level::ACCEPT;
        if (lvlCO > static_cast<int>(worst))
            worst = clsCO(out.co_ppm);
        if (lvlNO2 > static_cast<int>(worst))
            worst = clsNO2(out.no2_ppm);
        if (lvlNH3 > static_cast<int>(worst))
            worst = clsNH3(out.nh3_ppm);
        if (lvlVOC > static_cast<int>(worst))
            worst = clsVOC(out.voc_ppb);
        out.status = toStr(worst);
    }
    else
    {
        int score = 3 * lvlCO + 2 * lvlNO2 + lvlNH3 + lvlVOC;
        out.status = (score <= 2) ? "Acceptable" : (score <= 5) ? "Elevated"
                                                                : "Dangerous";
    }
    pub_->publish(out);
}

IGNITION_ADD_PLUGIN(GasSensorPlugin,
                    System,
                    GasSensorPlugin::ISystemConfigure,
                    GasSensorPlugin::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(GasSensorPlugin, "gas_sensor")
