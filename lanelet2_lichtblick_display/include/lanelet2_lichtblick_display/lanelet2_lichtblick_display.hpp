#pragma once

#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "lanelet2_map_interface/lanelet2_map_interface.hpp"


namespace lanelet2_lichtblick_display {

template <typename C> struct is_vector : std::false_type {};    
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};    
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;


/**
 * @brief Lanelet2LichtblickDisplay class
 */
class Lanelet2LichtblickDisplay : public rclcpp::Node {

 public:

  Lanelet2LichtblickDisplay();

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
   */
  template <typename T>
  void declareAndLoadParameter(const std::string &name,
                               T &param,
                               const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief Check for a new or changed Lanelet2 map and publish markers if present
   *
   * This function queries the LL2MapInterface for the current map pointer and compares it
   * against the internally stored `last_map_ptr_`. If a new map is detected (pointer change
   * and non-null), it triggers publishMarker(...) to convert the map into visualization markers.
   *
   * Called periodically by the node's timer.
   */
  void checkMapStatus();


  /**
   * @brief Convert a hex color string into normalized RGB float components.
   *
   * Accepted formats: `"#RRGGBB"`, `"0xRRGGBB"`, or `"RRGGBB"`. If the input is invalid
   * (wrong length or parse error), the function writes black (0,0,0) into r,g,b and logs
   * a warning.
   */
  void hexToRgb(const std::string& hex, float& r, float& g, float& b);

  /**
   * @brief Converts a Lanelet2 map into a MarkerArray and publishes it.
   *
   * This function iterates over lanelets in `lanelet_map` and creates markers for the various elements.
   * The marker array is published on the `marker_array_publisher_` which is configured with
   * transient_local QoS so late joiners receive the last message.
   *
   * @param lanelet_map lanelet map to visualize
   */
  void publishMarker(const lanelet::LaneletMapConstPtr& lanelet_map);

  /**
   * @brief Converts an Eigen::Vector3d into a ROS geometry_msgs::msg::Point
   *
   * Simple helper to copy x, y, z components from Eigen to the ROS message.
   *
   * @param point Eigen 3D point
   * @return geometry_msgs::msg::Point ROS point message
   */
  geometry_msgs::msg::Point toRos(const Eigen::Vector3d &point);

  /**
   * @brief Extracts the reference/effect line of a regulatory element.
   *
   * Only the first reference line of the regulatory element is considered.
   * Only the end points of that reference line are considered.
   *
   * @param[in] regulatory_element regulatory element
   * @return reference line
   */
  std::optional<std::array<geometry_msgs::msg::Point, 2>> regulatoryElementReferenceLine(
      const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);

  /**
   * @brief Extracts the sign/signal positions of a regulatory element.
   *
   * Only the first point of referenced line strings is considered.
   *
   * @param[in] regulatory_element regulatory element
   * @return positions
   */
  std::vector<geometry_msgs::msg::Point> regulatoryElementPositions(
      const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);


 private:

  /**
   * @brief Auto-reconfigurable parameters for dynamic reconfiguration
   */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;

  /**
   * @brief Callback handle for dynamic parameter reconfiguration
   */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  /**
   * @brief Publisher Marker Array
   */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<bool> need_republish_{false};

  // Store the last map pointer to detect changes
  lanelet::LaneletMapConstPtr last_map_ptr_; 

  // A counter to give unique IDs to markers
  int marker_id_counter_ = 0;

  std::shared_ptr<LL2MapInterface> ll2if_;


  // Parameters
  std::string output_topic_ = "/lichtblick_lanelet2_map";

  double left_bound_line_width_ = 0.1;
  std::string left_bound_color_hex_ = "#0000FF";
  double left_bound_line_opacity_ = 0.5;

  double right_bound_line_width_ = 0.1;
  std::string right_bound_color_hex_ = "#FF0000";
  double right_bound_line_opacity_ = 0.5;

  double centerline_line_width_ = 0.08;
  std::string centerline_color_hex_ = "#008000";
  double centerline_line_opacity_ = 0.4;

  double reference_line_width_ = 0.2;
  std::string reference_line_color_hex_ = "#FFFF00";
  double reference_line_opacity_ = 0.5;

  std::string traffic_light_mesh_resource_;
  double traffic_light_scale_ = 1.0;
  double traffic_light_z_offset_ = 1.3;
  double traffic_light_opacity_ = 1.0;

  std::string yield_sign_mesh_resource_;
  double yield_sign_scale_ = 1.0;
  double yield_sign_z_offset_ = 0.0;
  double yield_sign_opacity_ = 1.0;

};


}
