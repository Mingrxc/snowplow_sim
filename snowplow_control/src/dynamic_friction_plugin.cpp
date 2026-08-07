#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace snowplow_control
{

class DynamicFrictionPlugin final : public gazebo::ModelPlugin
{
public:
  DynamicFrictionPlugin() = default;

  ~DynamicFrictionPlugin() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    link_name_ = read_string(sdf, "link_name", "dynamic_friction_link");
    collision_name_ = read_string(sdf, "collision_name", "dynamic_friction_collision");
    command_topic_ = read_string(sdf, "command_topic", "/dynamic_friction/set");
    state_topic_ = read_string(sdf, "state_topic", "/dynamic_friction/state");
    mu_.store(read_double(sdf, "initial_mu", 0.80));
    mu2_.store(read_double(sdf, "initial_mu2", 0.50));
    publish_rate_ = std::max(0.1, read_double(sdf, "publish_rate", 2.0));

    if (!resolve_collision()) {
      gzerr << "[dynamic_friction_plugin] Could not resolve collision "
            << collision_name_ << " on link " << link_name_ << "\n";
      return;
    }

    int argc = 0;
    char ** argv = nullptr;
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("dynamic_friction_pad_plugin");
    state_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      state_topic_, 10);
    command_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_,
      10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        if (message->data.size() < 2) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Ignoring dynamic friction command with fewer than two values");
          return;
        }
        set_friction(
          std::max(0.0, message->data[0]),
          std::max(0.0, message->data[1]));
      });
    timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      [this]() { publish_state(); });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    set_friction(mu_.load(), mu2_.load());
    gzmsg << "[dynamic_friction_plugin] Ready on " << model_->GetName()
          << ", command_topic=" << command_topic_
          << ", state_topic=" << state_topic_
          << ", initial mu=" << mu_.load()
          << ", mu2=" << mu2_.load() << "\n";
  }

private:
  static std::string read_string(
    const sdf::ElementPtr & sdf,
    const std::string & key,
    const std::string & fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
  }

  static double read_double(
    const sdf::ElementPtr & sdf,
    const std::string & key,
    double fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback;
  }

  bool resolve_collision()
  {
    if (!model_) {
      return false;
    }
    auto link = model_->GetLink(link_name_);
    if (!link) {
      return false;
    }
    collision_ = link->GetCollision(collision_name_);
    return static_cast<bool>(collision_);
  }

  void set_friction(double mu, double mu2)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!collision_) {
      return;
    }
    auto surface = collision_->GetSurface();
    if (!surface) {
      return;
    }
    auto friction = surface->FrictionPyramid();
    if (!friction) {
      return;
    }
    friction->SetMuPrimary(mu);
    friction->SetMuSecondary(mu2);
    mu_.store(mu);
    mu2_.store(mu2);
    publish_state_unlocked();
  }

  void publish_state()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_state_unlocked();
  }

  void publish_state_unlocked()
  {
    if (!state_publisher_) {
      return;
    }
    std_msgs::msg::Float64MultiArray message;
    message.data = {mu_.load(), mu2_.load()};
    state_publisher_->publish(message);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::CollisionPtr collision_;
  std::string link_name_;
  std::string collision_name_;
  std::string command_topic_;
  std::string state_topic_;
  double publish_rate_{2.0};

  std::atomic<double> mu_{0.80};
  std::atomic<double> mu2_{0.50};
  std::mutex mutex_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

}  // namespace snowplow_control

GZ_REGISTER_MODEL_PLUGIN(snowplow_control::DynamicFrictionPlugin)
