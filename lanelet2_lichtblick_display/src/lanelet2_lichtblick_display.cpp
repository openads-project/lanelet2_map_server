// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <lanelet2_lichtblick_display/lanelet2_lichtblick_display.hpp>

namespace lanelet2_lichtblick_display {

Lanelet2LichtblickDisplay::Lanelet2LichtblickDisplay() : Node("lanelet2_lichtblick_display") {
  this->declareAndLoadParameter("left_bound_line_width", left_bound_line_width_, "Width of the left boundary lines", true, false,
                                false);
  this->declareAndLoadParameter("left_bound_color_hex", left_bound_color_hex_, "Color of the left boundary lines", true, false,
                                false);
  this->declareAndLoadParameter("left_bound_line_opacity", left_bound_line_opacity_, "Opacity of the left boundary lines", true,
                                false, false);

  this->declareAndLoadParameter("right_bound_line_width", right_bound_line_width_, "Width of the right boundary lines", true,
                                false, false);
  this->declareAndLoadParameter("right_bound_color_hex", right_bound_color_hex_, "Color of the right boundary lines", true, false,
                                false);
  this->declareAndLoadParameter("right_bound_line_opacity", right_bound_line_opacity_, "Opacity of the right boundary lines",
                                true, false, false);

  this->declareAndLoadParameter("centerline_line_width", centerline_line_width_, "Width of the centerlines", true, false, false);
  this->declareAndLoadParameter("centerline_color_hex", centerline_color_hex_, "Color of the centerlines", true, false, false);
  this->declareAndLoadParameter("centerline_line_opacity", centerline_line_opacity_, "Opacity of the centerlines", true, false,
                                false);

  this->declareAndLoadParameter("lanelet_text_scale", lanelet_text_scale_, "Scale of the lanelet ID text", true, false, false);
  this->declareAndLoadParameter("lanelet_text_color", lanelet_text_color_hex_, "Color of the lanelet ID text", true, false,
                                false);
  this->declareAndLoadParameter("lanelet_text_opacity", lanelet_text_opacity_, "Opacity of the lanelet ID text", true, false,
                                false);

  this->declareAndLoadParameter("reference_line_width", reference_line_width_, "Width of the reference lines", true, false,
                                false);
  this->declareAndLoadParameter("reference_line_color_hex", reference_line_color_hex_, "Color of the reference lines", true,
                                false, false);
  this->declareAndLoadParameter("reference_line_opacity", reference_line_opacity_, "Opacity of the reference lines", true, false,
                                false);

  this->declareAndLoadParameter("traffic_light_mesh_resource", traffic_light_mesh_resource_,
                                "Link to the traffic light model to use", true, false, false);
  this->declareAndLoadParameter("traffic_light_scale", traffic_light_scale_, "Scale of the traffic lights models", true, false,
                                false);
  this->declareAndLoadParameter("traffic_light_z_offset", traffic_light_z_offset_,
                                "Offset in z-direction of the traffic lights models (depends on model and scale)", true, false,
                                false);
  this->declareAndLoadParameter("traffic_light_opacity", traffic_light_opacity_, "Opacity of the traffic lights", true, false,
                                false);

  this->declareAndLoadParameter("yield_sign_mesh_resource", yield_sign_mesh_resource_, "Link to the yield sign model to use",
                                true, false, false);
  this->declareAndLoadParameter("yield_sign_scale", yield_sign_scale_, "Scale of the yield sign models", true, false, false);
  this->declareAndLoadParameter("yield_sign_z_offset", yield_sign_z_offset_,
                                "Offset in z-direction of the yield sign models (depends on model and scale)", true, false,
                                false);
  this->declareAndLoadParameter("yield_sign_opacity", yield_sign_opacity_, "Opacity of the yield signs", true, false, false);

  this->setup();
}

template <typename T>
void Lanelet2LichtblickDisplay::declareAndLoadParameter(const std::string& name,
                                                        T& param,
                                                        const std::string& description,
                                                        const bool add_to_auto_reconfigurable_params,
                                                        const bool is_required,
                                                        const bool read_only,
                                                        const std::optional<double>& from_value,
                                                        const std::optional<double>& to_value,
                                                        const std::optional<double>& step_value,
                                                        const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr (is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

rcl_interfaces::msg::SetParametersResult Lanelet2LichtblickDisplay::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
        need_republish_.store(true);
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

void Lanelet2LichtblickDisplay::setup() {
  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&Lanelet2LichtblickDisplay::parametersCallback, this, std::placeholders::_1));

  // publisher for visualization markers
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/lichtblick_lanelet2_map", qos);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s' with transient_local QoS", marker_array_publisher_->get_topic_name());

  ll2if_ = std::make_shared<LL2MapInterface>(*this, "lanelet2_map_server");

  // periodically check for map updates
  timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&Lanelet2LichtblickDisplay::checkMapStatus, this));
}

void Lanelet2LichtblickDisplay::hexToRgb(const std::string& hex, float& r, float& g, float& b) {
  std::string clean_hex = hex;
  if (clean_hex.rfind("0x", 0) == 0) {
    clean_hex = clean_hex.substr(2);
  } else if (clean_hex.rfind("#", 0) == 0) {
    clean_hex = clean_hex.substr(1);
  }
  if (clean_hex.length() != 6) {
    RCLCPP_WARN(this->get_logger(), "Invalid hex color string: %s. Defaulting to black.", hex.c_str());
    r = g = b = 0.0f;
    return;
  }

  unsigned int r_int = 0U;
  unsigned int g_int = 0U;
  unsigned int b_int = 0U;

  std::istringstream rs(clean_hex.substr(0, 2));
  rs >> std::hex >> r_int;
  r = static_cast<float>(r_int) / 255.0f;

  std::istringstream gs(clean_hex.substr(2, 2));
  gs >> std::hex >> g_int;
  g = static_cast<float>(g_int) / 255.0f;

  std::istringstream bs(clean_hex.substr(4, 2));
  bs >> std::hex >> b_int;
  b = static_cast<float>(b_int) / 255.0f;
}

geometry_msgs::msg::Point Lanelet2LichtblickDisplay::toRos(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = point.x();
  ros_point.y = point.y();
  ros_point.z = point.z();
  return ros_point;
}

void Lanelet2LichtblickDisplay::clearAllMarkers() {
  visualization_msgs::msg::MarkerArray marker_array_msg;
  visualization_msgs::msg::Marker delete_marker;

  delete_marker.header.frame_id = map_frame_id;
  delete_marker.header.stamp = this->now();
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  marker_array_msg.markers.push_back(delete_marker);
  marker_array_publisher_->publish(marker_array_msg);
}

void Lanelet2LichtblickDisplay::addLineStripMarker(visualization_msgs::msg::MarkerArray& marker_array_msg,
                                                   int& current_marker_id,
                                                   const std::string& ns,
                                                   double line_width,
                                                   const std::string& color_hex,
                                                   double opacity,
                                                   const std_msgs::msg::Header& header,
                                                   const std::vector<geometry_msgs::msg::Point>& points) {
  if (points.empty()) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = current_marker_id++;                                   // give every marker a unique ID
  marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(0));  // indefinite since QoS is set to transient local
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = line_width;

  float r = 0.0F;
  float g = 0.0F;
  float b = 0.0F;
  hexToRgb(color_hex, r, g, b);
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = static_cast<float>(opacity);
  marker.points = points;

  marker_array_msg.markers.push_back(marker);
}

void Lanelet2LichtblickDisplay::addMeshMarker(visualization_msgs::msg::MarkerArray& marker_array_msg,
                                              int& current_marker_id,
                                              const std::string& ns,
                                              const std::string& mesh_resource,
                                              const geometry_msgs::msg::Point& position,
                                              double scale,
                                              double z_offset,
                                              double opacity,
                                              const std_msgs::msg::Header& header,
                                              double yaw_ref_line,
                                              const geometry_msgs::msg::Point& middle_point) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = current_marker_id++;                                   // give every marker a unique ID
  marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(0));  // indefinite since QoS is set to transient local
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_resource;
  // uniform scale
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  // set the position of the mesh marker (depends on scale and model)
  geometry_msgs::msg::Point adjusted_position = position;
  adjusted_position.z = z_offset;
  marker.pose.position = adjusted_position;

  // calculate orientation orthogonal to the reference line
  // 1. need vector from traffic sign to the reference line
  double dx_to_ref = middle_point.x - position.x;
  double dy_to_ref = middle_point.y - position.y;
  double yaw_to_ref = std::atan2(dy_to_ref, dx_to_ref);
  // 2. two possible perpendicular orientations to the reference line
  double cand1 = yaw_ref_line + M_PI_2;  // +90°
  double cand2 = yaw_ref_line - M_PI_2;  // -90°
  // measure angular difference (normalized) using atan2(sin,cos)
  double d1 = std::abs(std::atan2(std::sin(yaw_to_ref - cand1), std::cos(yaw_to_ref - cand1)));
  double d2 = std::abs(std::atan2(std::sin(yaw_to_ref - cand2), std::cos(yaw_to_ref - cand2)));
  double final_yaw = (d1 < d2) ? cand1 : cand2;

  tf2::Quaternion q;
  q.setRPY(0, 0, final_yaw + M_PI_2);
  marker.pose.orientation = tf2::toMsg(q);

  marker.mesh_use_embedded_materials = true;
  marker.color.a = static_cast<float>(opacity);

  marker_array_msg.markers.push_back(marker);
}

std::optional<std::array<geometry_msgs::msg::Point, 2>> Lanelet2LichtblickDisplay::regulatoryElementReferenceLine(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  const std::vector<lanelet::ConstLineString3d> reference_lines =
      regulatory_element->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
  if (reference_lines.empty()) {
    return std::nullopt;
  }
  const std::vector<Eigen::Vector3d> reference_line = reference_lines.front().basicLineString();
  if (reference_line.size() < 2) {
    return std::nullopt;
  }
  std::array<geometry_msgs::msg::Point, 2> reference_line_ros = {toRos(reference_line.front()), toRos(reference_line.back())};
  return reference_line_ros;
}

std::vector<geometry_msgs::msg::Point> Lanelet2LichtblickDisplay::regulatoryElementPositions(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  std::vector<geometry_msgs::msg::Point> positions;
  const std::vector<lanelet::ConstLineString3d> sign_lines =
      regulatory_element->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers);
  for (const auto& const_sign_line : sign_lines) {
    const std::vector<Eigen::Vector3d> sign_line = const_sign_line.basicLineString();
    if (!sign_line.empty()) {
      positions.push_back(toRos(sign_line.front()));
    }
  }
  return positions;
}

void Lanelet2LichtblickDisplay::checkMapStatus() {
  lanelet::LaneletMapConstPtr current_map_ptr = ll2if_->getMapPtr();

  // Check if the map has been loaded or if a new map has been received
  if (current_map_ptr && current_map_ptr != last_map_ptr_) {
    RCLCPP_INFO(this->get_logger(), "New Lanelet2 map detected. Publishing markers...");

    map_frame_id = ll2if_->map_frame_id_;
    this->clearAllMarkers();

    this->publishMarker(current_map_ptr);
    last_map_ptr_ = current_map_ptr;
    need_republish_.store(false);  // cleared after publish
  }

  // Check if parameters changed
  if (need_republish_.load() && last_map_ptr_) {
    RCLCPP_INFO(this->get_logger(), "Parameters changed. Republishing markers...");
    this->publishMarker(last_map_ptr_);
    need_republish_.store(false);
  }
}

void Lanelet2LichtblickDisplay::publishMarker(const lanelet::LaneletMapConstPtr& lanelet_map) {
  if (lanelet_map->laneletLayer.empty()) {
    RCLCPP_WARN(this->get_logger(), "Lanelet2 map is empty.");
    return;
  }
  visualization_msgs::msg::MarkerArray marker_array_msg;
  int current_marker_id = 0;
  rclcpp::Time current_timestamp = this->now();

  std_msgs::msg::Header header;
  header.frame_id = map_frame_id;
  header.stamp = current_timestamp;
  float r = 0.0F;
  float g = 0.0F;
  float b = 0.0F;

  // Iterate through all lanelets in the map
  for (const auto& lanelet : lanelet_map->laneletLayer) {
    // --- Visualize Lines ---
    std::vector<geometry_msgs::msg::Point> left_points, right_points, centerline_points;
    for (const auto& p : lanelet.leftBound()) left_points.push_back(toRos(p.basicPoint()));
    for (const auto& p : lanelet.rightBound()) right_points.push_back(toRos(p.basicPoint()));
    for (const auto& p : lanelet.centerline()) centerline_points.push_back(toRos(p.basicPoint()));

    addLineStripMarker(marker_array_msg, current_marker_id, "lanelet_left_boundaries", left_bound_line_width_,
                       left_bound_color_hex_, left_bound_line_opacity_, header, left_points);
    addLineStripMarker(marker_array_msg, current_marker_id, "lanelet_right_boundaries", right_bound_line_width_,
                       right_bound_color_hex_, right_bound_line_opacity_, header, right_points);
    addLineStripMarker(marker_array_msg, current_marker_id, "lanelet_centerlines", centerline_line_width_, centerline_color_hex_,
                       centerline_line_opacity_, header, centerline_points);

    // --- Visualize Lanelet ID Text ---
    if (!lanelet.centerline().empty()) {
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = map_frame_id;
      text_marker.header.stamp = current_timestamp;
      text_marker.ns = "lanelet_ids";
      text_marker.id = current_marker_id++;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(0));
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

      // position the text at the geometric center of the lanelet's centerline
      geometry_msgs::msg::Point centerline_center;
      double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
      for (const auto& p : lanelet.centerline()) {
        sum_x += p.x();
        sum_y += p.y();
        sum_z += p.z();
      }
      if (!lanelet.centerline().empty()) {
        const auto centerline_size = static_cast<double>(lanelet.centerline().size());
        centerline_center.x = sum_x / centerline_size;
        centerline_center.y = sum_y / centerline_size;
        centerline_center.z = sum_z / centerline_size;
      }
      text_marker.pose.position = centerline_center;
      text_marker.scale.z = lanelet_text_scale_;  // For TEXT_VIEW_FACING, scale.z controls the character height

      hexToRgb(lanelet_text_color_hex_, r, g, b);
      text_marker.color.r = r;
      text_marker.color.g = g;
      text_marker.color.b = b;
      text_marker.color.a = static_cast<float>(lanelet_text_opacity_);

      text_marker.text = std::to_string(lanelet.id());

      marker_array_msg.markers.push_back(text_marker);
    }

    const auto& regulatory_elements = lanelet.regulatoryElements();
    for (const auto& regulatory_element : regulatory_elements) {
      // Check if the element is a reference line
      if (auto reference_line = regulatoryElementReferenceLine(regulatory_element)) {
        std::vector<geometry_msgs::msg::Point> ref_points = {reference_line->at(0), reference_line->at(1)};
        addLineStripMarker(marker_array_msg, current_marker_id, "reference_lines", reference_line_width_,
                           reference_line_color_hex_, reference_line_opacity_, header, ref_points);

        // Common regulatory element logic
        // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/RegulatoryElementTagging.md
        std::string subtype = regulatory_element->attribute("subtype").value();
        double dx = reference_line->at(1).x - reference_line->at(0).x;
        double dy = reference_line->at(1).y - reference_line->at(0).y;
        double yaw_ref_line = std::atan2(dy, dx);
        geometry_msgs::msg::Point middle_point;
        middle_point.x = (reference_line->at(0).x + reference_line->at(1).x) / 2.0;
        middle_point.y = (reference_line->at(0).y + reference_line->at(1).y) / 2.0;

        std::vector<geometry_msgs::msg::Point> positions = regulatoryElementPositions(regulatory_element);
        for (const auto& p : positions) {
          if (subtype == "traffic_light") {
            addMeshMarker(marker_array_msg, current_marker_id, "traffic_lights", traffic_light_mesh_resource_, p,
                          traffic_light_scale_, traffic_light_z_offset_, traffic_light_opacity_, header, yaw_ref_line,
                          middle_point);
          } else if (subtype == "right_of_way") {
            addMeshMarker(marker_array_msg, current_marker_id, "yield_signs", yield_sign_mesh_resource_, p, yield_sign_scale_,
                          yield_sign_z_offset_, yield_sign_opacity_, header, yaw_ref_line, middle_point);
          }
        }
      }
    }
  }

  // Publish the marker array
  marker_array_publisher_->publish(marker_array_msg);
}

}  // namespace lanelet2_lichtblick_display

/**
 * @brief Starts the Lanelet2 Lichtblick display node.
 *
 * @param argc number of command-line arguments
 * @param argv command-line argument array
 * @return process exit code
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lanelet2_lichtblick_display::Lanelet2LichtblickDisplay>());
  rclcpp::shutdown();

  return 0;
}
