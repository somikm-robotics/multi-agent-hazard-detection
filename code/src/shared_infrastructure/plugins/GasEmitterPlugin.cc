#include "GasEmitterPlugin.hh"
#include <ignition/plugin/Register.hh>
#include <ignition/math/Rand.hh>
#include <cmath>
#include <chrono>

using namespace ignition;
using namespace gazebo;

using WorldPose = components::WorldPose;

// helper: sine‑wave plus noise
static double sinNoise(double t, double lo, double hi, double period, double f)
{
    double mid = 0.5 * (hi + lo), amp = 0.5 * (hi - lo);
    return mid + amp * std::sin(2.0 * M_PI * t / period) + amp * f * ignition::math::Rand::DblUniform(-1, 1);
}

//------------------------------------------------------------------
void GasEmitterPlugin::Configure(const Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 EntityComponentManager &,
                                 EventManager &)
{
    if (_sdf->HasElement("topic"))
        topic_ = _sdf->Get<std::string>("topic");
    if (_sdf->HasElement("rate"))
        rate_ = _sdf->Get<double>("rate");
    if (_sdf->HasElement("noise"))
        noise_ = _sdf->Get<double>("noise");

    selfEnt_ = _entity; // remember own model

    // Draw *once* a baseline concentration for this hazard
    co0_ = Uniform(2.0, 20.0);     // ppm
    nh30_ = Uniform(1.0, 10.0);    // ppm
    no20_ = Uniform(0.2, 3.0);     // ppm
    voc0_ = Uniform(150.0, 600.0); // ppb

    // ───── defer ROS node creation until ROS is already initialised ─────
    //   try { rclcpp::init(0,nullptr); } catch(...) {}
    //   node_ = rclcpp::Node::make_shared("gas_emitter");
    //   pub_  = node_->create_publisher<std_msgs::msg::Float32MultiArray>(topic_,10);
    //   std::thread([](rclcpp::Node::SharedPtr n){ rclcpp::spin(n); }, node_).detach();
}

//------------------------------------------------------------------
void GasEmitterPlugin::PreUpdate(const UpdateInfo &_info,
                                 EntityComponentManager &ecm)
{
    if (!this->node_)
    {
        if (!rclcpp::ok()) // ← use ok() instead of is_initialized()
        {
            int argc = 1;
            const char *argv[] = {"gas_emitter"};
            try
            {
                rclcpp::init(argc, const_cast<char **>(argv));
            }
            catch (const std::runtime_error &e)
            {
                // Log and re‑throw so we SEE any unusual init clash
                ignerr << "[GasEmitterPlugin] ROS init failed: "
                       << e.what() << std::endl;
                throw; // let Ignition abort world load
            }
        }

        // now ROS is up – create our node & publisher once
        this->node_ = rclcpp::Node::make_shared("gas_emitter");
        this->pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(topic_, 10);

        ignmsg << "[GasEmitterPlugin] ROS node + publisher created on topic "
               << topic_ << std::endl;
    }

    double now = std::chrono::duration<double>(_info.simTime).count();
    if (now - last_pub_ < 1.0 / rate_)
        return;

    // tiny ±noise around the fixed baseline each tick
    auto jitter = [this]()
    {
        return 1.0 + noise_ * ignition::math::Rand::DblUniform(-1, 1);
    };

    /* ─── GasEmitterPlugin.hh ──────────────────────────────── */
    ignition::math::Vector3d pos;
    if (auto wp = ecm.Component<WorldPose>(selfEnt_))
        pos = wp->Data().Pos(); // valid after 1–2 iterations
    else if (auto rp = ecm.Component<components::Pose>(selfEnt_))
    {
        // static models always have this from tick 0
        pos = rp->Data().Pos();
    }
    else
    {
        // Neither pose is available yet – try next iteration
        return;
    }

    std_msgs::msg::Float32MultiArray m;
    m.data = {
        static_cast<float>(co0_ * jitter()),  // CO   (ppm)
        static_cast<float>(nh30_ * jitter()), // NH3  (ppm)
        static_cast<float>(no20_ * jitter()), // NO2  (ppm)
        static_cast<float>(voc0_ * jitter()), // VOC  (ppb)
        static_cast<float>(pos.X()),          // index 4
        static_cast<float>(pos.Y()),          // index 5
        static_cast<float>(pos.Z())           // index 6
    };
    pub_->publish(m);
    last_pub_ = now;
}

IGNITION_ADD_PLUGIN(GasEmitterPlugin,
                    System,
                    GasEmitterPlugin::ISystemConfigure,
                    GasEmitterPlugin::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(GasEmitterPlugin, "gas_emitter")
