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
 */

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <cmath>

class TwistMarker {
public:
  TwistMarker(double scale = 1.0, double z = 0.0,
              const std::string &frame_id = "base_link")
      : frame_id_(frame_id), scale_(scale), z_(z) {
    // ID and type:
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::ARROW;

    // Frame ID:
    marker_.header.frame_id = frame_id_;

    // Pre-allocate points for setting the arrow with the twist:
    marker_.points.resize(2);

    // Vertical position:
    marker_.pose.position.z = z_;

    // Scale:
    marker_.scale.x = 0.05 * scale_;
    marker_.scale.y = 2 * marker_.scale.x;

    // Color:
    marker_.color.a = 1.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;
  }

  void update(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    marker_.points[1].x = msg->speed;
    marker_.points[1].y = msg->steering_angle;
  }

  const visualization_msgs::msg::Marker &getMarker() { return marker_; }

private:
  visualization_msgs::msg::Marker marker_;

  std::string frame_id_;
  double scale_;
  double z_;
};

class TwistMarkerPublisher : public rclcpp::Node {
public:
  TwistMarkerPublisher(double scale = 1.0, double z = 0.0) :
      rclcpp::Node("twist_marker"),
      marker_(scale, z)
  {

    pub_ = create_publisher<visualization_msgs::msg::Marker>("twist_marker", 1);
    sub_ = create_subscription<ackermann_msgs::msg::AckermannDrive> (
      "cmd_vel_out",
      1,
      std::bind(&TwistMarkerPublisher::callback, this, std::placeholders::_1)
    );
  }

  void callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    marker_.update(msg);

    pub_->publish(marker_.getMarker());
  }

private:
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;

  TwistMarker marker_;
};

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::shared_ptr<TwistMarkerPublisher> marker = std::make_shared<TwistMarkerPublisher>(1.0, 2.0);

  rclcpp::spin(marker);
  rclcpp::shutdown();

  // TwistMarkerPublisher t(1.0, 2.0);

  // while (ros::ok()) {
  //   ros::spin();
  // }

  return EXIT_SUCCESS;


  // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  // rclcpp::init(argc, argv);
  // std::shared_ptr<twist_mux::TwistMux> mux = std::make_shared<twist_mux::TwistMux>();
  // mux->init(mux);
  // rclcpp::spin(mux);
  // rclcpp::shutdown();

  // return 0;
}
