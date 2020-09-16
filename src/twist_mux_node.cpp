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

// Force flush of the stdout buffer.
// This ensures a correct sync of all prints
// even when executed simultaneously within the launch file.


#include <twist_mux/twist_mux.hpp>

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::shared_ptr<twist_mux::TwistMux> mux = std::make_shared<twist_mux::TwistMux>();
  mux->init();
  rclcpp::spin(mux);
  rclcpp::shutdown();

  return 0;
}

