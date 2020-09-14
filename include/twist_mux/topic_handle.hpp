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

#ifndef TOPIC_HANDLE_H
#define TOPIC_HANDLE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

#include <twist_mux/utils.hpp>
#include <twist_mux/twist_mux.hpp>

#include <boost/utility.hpp>
#include <memory>

#include <string>
#include <vector>

namespace twist_mux
{

template<typename T>
class TopicHandle_ :public boost::noncopyable
{
public:

  typedef int priority_type;

  /**
   * @brief TopicHandle_
   * @param name Name identifier
   * @param topic Topic name
   * @param timeout Timeout to consider that the messages are old; note
   * that initially the message stamp is set to 0.0, so the message has
   * expired
   * @param priority Priority of the topic
   */
  TopicHandle_(std::shared_ptr<rclcpp::Node>& node, const std::string& name, const std::string& topic, double timeout, priority_type priority, TwistMux* mux)
    : nh_(node)
    , name_(name)
    , topic_(topic)
    , timeout_(timeout)
    , priority_(clamp(priority, priority_type(0), priority_type(255)))
    , mux_(mux)
    , stamp_(nh_->now())
  {
    RCLCPP_INFO_STREAM
    (
      nh_->get_logger(),
      "Topic handler '" << name_ << "' subscribed to topic '" << topic_ <<
      "': timeout = " << ((timeout_) ? std::to_string(timeout_) + "s" : "None") <<
      ", priority = " << static_cast<int>(priority_)
    );
  }

  virtual ~TopicHandle_()
  {
    //TODO is there another way to complete this in ros2
    // subscriber_.shutdown();
  }

  /**
   * @brief hasExpired
   * @return true if the message has expired; false otherwise.
   *         If the timeout is set to 0.0, this function always returns
   *         false
   */
  bool hasExpired() const
  {
    return (timeout_ > 0.0) and
           ((nh_->now() - stamp_).nanoseconds() / 1e9 > timeout_);
  }

  const std::string& getName() const
  {
    return name_;
  }

  const std::string& getTopic() const
  {
    return topic_;
  }

  const double& getTimeout() const
  {
    return timeout_;
  }

  /**
   * @brief getPriority Priority getter
   * @return Priority
   */
  const priority_type& getPriority() const
  {
    return priority_;
  }

  const T& getStamp() const
  {
    return stamp_;
  }

  const T& getMessage() const
  {
    return msg_;
  }

protected:

  std::string name_;
  std::string topic_;
  typename rclcpp::Subscription<T>::SharedPtr subscriber_;
  double timeout_;
  priority_type priority_;

  TwistMux* mux_;

  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Time stamp_;
  T msg_;
};

class VelocityTopicHandle : public TopicHandle_<ackermann_msgs::msg::AckermannDrive>
{
private:
  typedef TopicHandle_<ackermann_msgs::msg::AckermannDrive> base_type;

public:
  typedef typename base_type::priority_type priority_type;

  VelocityTopicHandle(std::shared_ptr<rclcpp::Node>& node, const std::string& name, const std::string& topic, double timeout, priority_type priority, TwistMux* mux)
    : base_type(node, name, topic, timeout, priority, mux)
  {
    subscriber_ = nh_->create_subscription<ackermann_msgs::msg::AckermannDrive> (
      topic_,
      1,
      std::bind(&VelocityTopicHandle::callback, this, std::placeholders::_1)
    );
  }

  bool isMasked(priority_type lock_priority) const
  {
    return hasExpired() or (getPriority() < lock_priority);
  }

  void callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
  {
    stamp_ = nh_->now();
    msg_   = *msg;

    // Check if this twist has priority.
    // Note that we have to check all the locks because they might time out
    // and since we have several topics we must look for the highest one in
    // all the topic list; so far there's no O(1) solution.
    if (mux_->hasPriority(*this))
    {
      mux_->publishTwist(msg);
    }
  }
};

class LockTopicHandle : public TopicHandle_<std_msgs::msg::Bool>
{
private:
  typedef TopicHandle_<std_msgs::msg::Bool> base_type;

public:
  typedef typename base_type::priority_type priority_type;

  LockTopicHandle(std::shared_ptr<rclcpp::Node>& node, const std::string& name, const std::string& topic, double timeout, priority_type priority, TwistMux* mux)
    : base_type(node, name, topic, timeout, priority, mux)
  {
    subscriber_ = nh_->create_subscription<std_msgs::msg::Bool> (
      topic_,
      1,
      std::bind(&LockTopicHandle::callback, this, std::placeholders::_1)
    );
  }

  /**
   * @brief isLocked
   * @return true if has expired or locked (i.e. bool message data is true)
   */
  bool isLocked() const
  {
    return hasExpired() or getMessage().data;
  }

  void callback(const std_msgs::msg::Bool::ConstPtr msg)
  {
    stamp_ = nh_->now();
    msg_   = *msg;
  }
};

} // namespace twist_mux

#endif // TOPIC_HANDLE_H
