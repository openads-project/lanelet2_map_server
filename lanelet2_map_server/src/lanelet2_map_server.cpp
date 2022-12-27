#include "lanelet2_map_server/lanelet2_map_server.hpp"

LL2MapServer::LL2MapServer() : Node("ll2_map_server")
{
  // Initialize service to change the map-parameters
  change_map_srv_ = this->create_service<lanelet2_map_manager_srvs::srv::ChangeMapParams>("~/change_map_parameters", std::bind(&LL2MapServer::change_params, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready!");
}

void LL2MapServer::change_params(const std::shared_ptr<lanelet2_map_manager_srvs::srv::ChangeMapParams::Request> request, std::shared_ptr<lanelet2_map_manager_srvs::srv::ChangeMapParams::Response> response)
{
  map_filename_ = request->map_filename;
  map_frame_id_ = request->map_frame_id;
  origin_lat_ = request->origin_lat;
  origin_lon_ = request->origin_lon;

  // To-Do: Maybe add sanity check of map here, before creating service --> e.g. check if map is available etc.

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Set lanelet2-map to " << map_filename_ << "\n Origin Lat: " << origin_lat_ << "\n Origin Lon: " << origin_lon_);
  if(!params_set_)
  {
    params_set_ = true;
    provide_map_srv_ = this->create_service<lanelet2_map_manager_srvs::srv::ProvideMapParams>("~/provide_map_parameters", std::bind(&LL2MapServer::provide_params, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting up service to provide the lanelet2 parameters!");

  }
  response->success = params_set_;
}

void LL2MapServer::provide_params(const std::shared_ptr<lanelet2_map_manager_srvs::srv::ProvideMapParams::Request> request, std::shared_ptr<lanelet2_map_manager_srvs::srv::ProvideMapParams::Response> response)
{
  response->map_filename = map_filename_;
  response->map_frame_id = map_frame_id_;
  response->origin_lat = origin_lat_;
  response->origin_lon = origin_lon_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init!");
  rclcpp::spin(std::make_shared<LL2MapServer>());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spinning!");
  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown!");
  return 0;
}
