/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  or send a letter to
 *  Creative Commons, 444 Castro Street, Suite 900,
 *  Mountain View, California, 94041, USA.
 *********************************************************************/

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 */

#include <twist_mux/twist_mux.hpp>
#include <twist_mux/topic_handle.hpp>
#include <twist_mux/twist_mux_diagnostics.hpp>
#include <twist_mux/twist_mux_diagnostics_status.hpp>
#include <twist_mux/utils.hpp>
// #include <twist_mux/xmlrpc_helpers.h>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: linear (abs(x)) or angular (abs(yaw))
 * @param old_twist Old velocity
 * @param new_twist New velocity
 * @return true is any of the absolute velocity components has increased
 */
bool hasIncreasedAbsVelocity(const geometry_msgs::msg::Twist& old_twist, const geometry_msgs::msg::Twist& new_twist)
{
  const auto old_linear_x = std::abs(old_twist.linear.x);
  const auto new_linear_x = std::abs(new_twist.linear.x);

  const auto old_angular_z = std::abs(old_twist.angular.z);
  const auto new_angular_z = std::abs(new_twist.angular.z);

  return (old_linear_x  < new_linear_x ) or
         (old_angular_z < new_angular_z);
}

namespace twist_mux
{

TwistMux::TwistMux(std::shared_ptr<rclcpp::Node> node, int window_size) : rclcpp::Node("TWIST_MUX") , nh_(node)
{
  std::cout << "constructor\n";

  // nh_ = node;
  /// Get topics and locks:
  velocity_hs_ = std::make_shared<velocity_topic_container>();
  lock_hs_     = std::make_shared<lock_topic_container>();
  getTopicHandles("locks" , *lock_hs_ );

  getTopicHandles("topics", *velocity_hs_);

  /// Publisher for output topic:
  cmd_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 1);

  /// Diagnostics:
  diagnostics_ = std::make_shared<diagnostics_type>(nh_);
  status_      = std::make_shared<status_type>();
  status_->velocity_hs = velocity_hs_;
  status_->lock_hs     = lock_hs_;

  diagnostics_timer_ = nh_->create_wall_timer(
    std::chrono::microseconds(1000000/static_cast<int>(DIAGNOSTICS_FREQUENCY)),
    std::bind(&TwistMux::updateDiagnostics, this)
  );
  std::cout << "END constructor \n";

}

TwistMux::~TwistMux()
{
  std::cout << "Destructor \n";
}

void TwistMux::updateDiagnostics()
{
  std::cout << "Diagnostics \n";
  status_->priority = getLockPriority();
  diagnostics_->updateStatus(status_);
}

void TwistMux::publishTwist(const geometry_msgs::msg::Twist::ConstPtr& msg)
{
  cmd_pub_->publish(*msg);
}

// TODO rewrite
template<typename T>
void TwistMux::getTopicHandles(const std::string& param_name, std::list<T>& topic_hs)
{
  std::cout << param_name << std::endl;

  for (int i = 0; i < 100; i++)
  {
    std::string name = "";
    std::string topic = "";
    double timeout = -1.0;
    int priority = -1;
    nh_->declare_parameter(param_name + "." + std::to_string(i) + ".name");
    nh_->declare_parameter(param_name + "." + std::to_string(i) + ".topic");
    nh_->declare_parameter(param_name + "." + std::to_string(i) + ".timeout");
    nh_->declare_parameter(param_name + "." + std::to_string(i) + ".priority");

    nh_->get_parameter(param_name + "." + std::to_string(i) + ".name", name);
    nh_->get_parameter(param_name + "." + std::to_string(i) + ".topic", topic);
    nh_->get_parameter(param_name + "." + std::to_string(i) + ".timeout", timeout);
    nh_->get_parameter(param_name + "." + std::to_string(i) + ".priority", priority);
    std::cout << param_name + "." + std::to_string(i) + ".name"<< std::endl;
    std::cout << "name: " << name << std::endl;
    std::cout << "topic: " << topic << std::endl;
    std::cout << "timeout: " << timeout << std::endl;
    std::cout << "priority: " << priority << std::endl;
    std::cout << std::endl;
    if(name == "" || topic == "" || timeout == -1.0 || priority == -1)
    {
      break;
    }
    topic_hs.emplace_back(nh_, name, topic, timeout, priority, this);
  }


  // this->declare_parameter(param_name);



  // int test;
  // this->get_parameter(param_name, test);
  // std::cout << decltype(test) << std::endl;
  // std::cout << test << std::endl;


  // try
  // {
  //   xh::Array output;
  //   xh::fetchParam(nh_priv, param_name, output);

  //   xh::Struct output_i;
  //   std::string name, topic;
  //   double timeout;
  //   int priority;
  //   for (int i = 0; i < output.size(); ++i)
  //   {
  //     xh::getArrayItem(output, i, output_i);

  //     xh::getStructMember(output_i, "name"    , name    );
  //     xh::getStructMember(output_i, "topic"   , topic   );
  //     xh::getStructMember(output_i, "timeout" , timeout );
  //     xh::getStructMember(output_i, "priority", priority);

  //     topic_hs.emplace_back(this, name, topic, timeout, priority, this);
  //   }
  // }
  // catch (const xh::XmlrpcHelperException& e)
  // {
  //   ROS_FATAL_STREAM("Error parsing params: " << e.what());
  // }
}

int TwistMux::getLockPriority()
{
  LockTopicHandle::priority_type priority = 0;

  /// max_element on the priority of lock topic handles satisfying
  /// that is locked:
  for (const auto& lock_h : *lock_hs_)
  {
    if (lock_h.isLocked())
    {
      auto tmp = lock_h.getPriority();
      if (priority < tmp)
      {
        priority = tmp;
      }
    }
  }

  std::cout << "Priority = " << static_cast<int>(priority) << std::endl;

  return priority;
}

bool TwistMux::hasPriority(const VelocityTopicHandle& twist)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string velocity_name = "NULL";

  /// max_element on the priority of velocity topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto& velocity_h : *velocity_hs_)
  {
    if (not velocity_h.isMasked(lock_priority))
    {
      const auto velocity_priority = velocity_h.getPriority();
      if (priority < velocity_priority)
      {
        priority = velocity_priority;
        velocity_name = velocity_h.getName();
      }
    }
  }

  return twist.getName() == velocity_name;
}

} // namespace twist_mux
