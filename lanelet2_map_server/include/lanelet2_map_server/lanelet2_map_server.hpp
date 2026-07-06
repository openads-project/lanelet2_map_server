// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

struct Lanelet2MapMeta {
  std::string map_path;
  double min_lat = 0.0;
  double min_lon = 0.0;
  double max_lat = 0.0;
  double max_lon = 0.0;
  double diagonal_length = -1.0;
};

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

class Lanelet2MapServer : public rclcpp::Node {
 public:
  /**
   * @brief Creates the Lanelet2 map server node and declares its parameters.
   */
  Lanelet2MapServer();

 private:
  /**
         * @brief Declares and loads a ROS parameter
         *
         * @param name name
         * @param param parameter variable to load into
         * @param description description
         * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
         * @param is_required whether failure to load parameter will stop node
         * @param read_only set parameter to read-only
         * @param from_value parameter range minimum
         * @param to_value parameter range maximum
         * @param step_value parameter range step
         * @param additional_constraints additional constraints description
         * @return true if the parameter was already initialized from an external source
         */
  template <typename T>
  bool declareAndLoadParameter(const std::string& name,
                               T& param,
                               const std::string& description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double>& from_value = std::nullopt,
                               const std::optional<double>& to_value = std::nullopt,
                               const std::optional<double>& step_value = std::nullopt,
                               const std::string& additional_constraints = "");

  /**
         * @brief Handles reconfiguration when a parameter value is changed
         *
         * @param parameters parameters
         * @return parameter change result
         */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Loads the configured map file and caches its serialized contents.
   */
  void loadMapContents();

  /**
   * @brief Clears map-specific parameters after an invalid or unloaded map state.
   */
  void unsetMapParameters();

  /**
   * @brief Writes the current map state into this node's exported ROS parameters.
   */
  void updateMapParameters();

  /**
   * @brief Sets up timers, callbacks, publishers, and initial map loading.
   */
  void setup();

  /**
   * @brief Recursively scans a directory for available Lanelet2 map files.
   *
   * @param directory root directory to search
   * @param maps output vector that receives discovered map metadata
   */
  void find_available_maps(const std::string& directory, std::vector<Lanelet2MapMeta>& maps) const;

  /**
   * @brief Fills the derived metadata fields for a discovered map.
   *
   * @param map_meta metadata entry whose geographic bounds and size are updated
   */
  void derive_map_meta(Lanelet2MapMeta& map_meta) const;

  /**
   * @brief Derives a south-west origin latitude and longitude from a Lanelet2 map file.
   *
   * @param map_filepath path to the map file
   * @param origin_lat output latitude of the derived south-west origin
   * @param origin_lon output longitude of the derived south-west origin
   * @return `true` if the origin could be derived from the map bounds
   */
  bool deriveOriginFromMap(const std::string& map_filepath, double& origin_lat, double& origin_lon) const;

  /**
   * @brief Tries to load a map with the provided origin to verify that it is usable.
   *
   * @param map_filepath path to the map file
   * @param origin_lat latitude used for projection
   * @param origin_lon longitude used for projection
   * @return `true` if the map passes the sanity check
   */
  bool map_sanity_check(std::string map_filepath, double origin_lat, double origin_lon) const;

  /**
   * @brief Publishes the static transform from the derived UTM frame to the map frame.
   */
  void pub_tf() const;

  /**
   * @brief Derives the UTM zone and hemisphere from a geographic position.
   *
   * @param latitude latitude in degrees
   * @param longitude longitude in degrees
   * @param zone output UTM zone number
   * @param northp output hemisphere flag, `true` for northern hemisphere
   */
  static void derive_utm_zone(const double latitude, const double longitude, int& zone, bool& northp);

  /**
   * @brief Stores the latest GNSS fix for automatic map selection.
   *
   * @param msg incoming GPS fix message
   */
  void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Periodically reevaluates the best map when automatic selection is enabled.
   */
  void automaticMapUpdateTimerCallback();

  /**
         * @brief Auto-reconfigurable parameters for dynamic reconfiguration
         */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter&)>>> auto_reconfigurable_params_;

  /**
         * @brief Callback handle for dynamic parameter reconfiguration
         */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::TimerBase::SharedPtr one_shot_timer_;
  rclcpp::TimerBase::SharedPtr automatic_map_timer_;

  bool use_automatic_map_selection_ = false;

  std::string map_directory_ = "/data/maps/default-maps";
  std::vector<Lanelet2MapMeta> available_maps_;
  std::string map_filepath_;
  std::string map_frame_id_ = "map";
  std::string map_contents_;
  double origin_lat_;
  double origin_lon_;
  bool origin_lat_set_ = false;
  bool origin_lon_set_ = false;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_subscription_;
  double current_latitude_ = std::numeric_limits<double>::quiet_NaN();
  double current_longitude_ = std::numeric_limits<double>::quiet_NaN();
  bool gps_fix_received_ = false;
};
