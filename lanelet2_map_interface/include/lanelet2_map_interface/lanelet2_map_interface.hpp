// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include "rclcpp/rclcpp.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>

#include <string.h>
#include <fstream>
#include <iostream>
#include <limits>

using namespace std::chrono_literals;
class LL2MapInterface {
 public:
  /**
   * @brief Creates a client-side interface for retrieving Lanelet2 map parameters from a map server node.
   *
   * @param parent_node node that owns the parameter client and timers
   * @param map_server_name fully qualified name of the map server node
   */
  LL2MapInterface(rclcpp::Node& parent_node, std::string map_server_name);

  /**
   * @brief Returns the currently loaded immutable Lanelet2 map.
   *
   * @return shared pointer to the loaded map, or `nullptr` if no map is available
   */
  lanelet::LaneletMapConstPtr getMapPtr();

  /**
   * @brief Returns the currently loaded mutable Lanelet2 map.
   *
   * @return shared pointer to the loaded map, or `nullptr` if no map is available
   */
  lanelet::LaneletMapPtr getNonConstMapPtr();

  /**
   * @brief Returns the projector used to interpret the loaded map coordinates.
   *
   * @return projector configured from the map origin parameters
   */
  std::shared_ptr<lanelet::Projector> getProjectorPtr();
  bool map_loaded_ = false;
  bool update_pending_ = false;  // Flag indicating if the client node should update map
  std::string map_frame_id_;

 private:
  rclcpp::Node& parent_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> frame_id_callback_handle_, contents_callback_handle_,
      origin_lat_callback_handle_, origin_lon_callback_handle_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  std::string map_filepath_;

  std::vector<rclcpp::Parameter> map_params_;
  std::string map_contents_;
  double origin_lat_ = std::numeric_limits<double>::quiet_NaN();
  double origin_lon_ = std::numeric_limits<double>::quiet_NaN();

  lanelet::LaneletMapPtr mapPtr_;
  std::shared_ptr<lanelet::Projector> utmProjectorPtr_;

  std::string map_server_name_;
  bool params_declared_ = false;

  /**
   * @brief Updates one cached map parameter from a value received from the map server.
   *
   * @param param updated parameter value from the map server
   */
  void updateMapParam(rclcpp::Parameter param);

  /**
   * @brief Loads and parses the map from the currently cached parameters.
   *
   * @return `true` if the map could be created successfully
   */
  bool loadMap();

  /**
   * @brief Validates that the required map parameters are available and usable.
   *
   * @return `true` if the cached parameters are sufficient for loading a map
   */
  bool validateParams();

  /**
   * @brief Checks for the configured map server and, once available, requests initial parameters and installs subscriptions.
   */
  void findMapServer();

  /**
   * @brief Reacts to a single parameter event from the map server.
   *
   * @param p parameter that changed on the server
   */
  void updateParamsCallback(const rclcpp::Parameter& p);

  /**
   * @brief Handles the asynchronous response of an initial parameter fetch.
   *
   * @param future future containing the requested map server parameters
   */
  void serviceParamsCallback(std::shared_future<std::vector<rclcpp::Parameter>> future);
};
