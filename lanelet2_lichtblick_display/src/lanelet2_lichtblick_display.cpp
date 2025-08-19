#include <lanelet2_lichtblick_display/lanelet2_lichtblick_display.hpp>


namespace lanelet2_lichtblick_display {


Lanelet2LichtblickDisplay::Lanelet2LichtblickDisplay() : Node("lanelet2_lichtblick_display") {

  this->declareAndLoadParameter("left_bound_line_width", left_bound_line_width_, "Width of the left boundary lines", true, false, false);
  this->declareAndLoadParameter("left_bound_color_hex", left_bound_color_hex_, "Color of the left boundary lines", true, false, false);
  this->declareAndLoadParameter("left_bound_line_opacity", left_bound_line_opacity_, "Opacity of the left boundary lines", true, false, false);

  this->declareAndLoadParameter("right_bound_line_width", right_bound_line_width_, "Width of the right boundary lines", true, false, false);
  this->declareAndLoadParameter("right_bound_color_hex", right_bound_color_hex_, "Color of the right boundary lines", true, false, false);
  this->declareAndLoadParameter("right_bound_line_opacity", right_bound_line_opacity_, "Opacity of the right boundary lines", true, false, false);

  this->declareAndLoadParameter("centerline_line_width", centerline_line_width_, "Width of the centerlines", true, false, false);
  this->declareAndLoadParameter("centerline_color_hex", centerline_color_hex_, "Color of the centerlines", true, false, false);
  this->declareAndLoadParameter("centerline_line_opacity", centerline_line_opacity_, "Opacity of the centerlines", true, false, false);

  this->declareAndLoadParameter("reference_line_width", reference_line_width_, "Width of the reference lines", true, false, false);
  this->declareAndLoadParameter("reference_line_color_hex", reference_line_color_hex_, "Color of the reference lines", true, false, false);
  this->declareAndLoadParameter("reference_line_opacity", reference_line_opacity_, "Opacity of the reference lines", true, false, false);

  this->declareAndLoadParameter("traffic_light_mesh_resource", traffic_light_mesh_resource_, "Link to the traffic light model to use", true, false, false);
  this->declareAndLoadParameter("traffic_light_scale", traffic_light_scale_, "Scale of the traffic lights models", true, false, false);
  this->declareAndLoadParameter("traffic_light_z_offset", traffic_light_z_offset_, "Offset in z-direction of the traffic lights models (depends on model and scale)", true, false, false);
  this->declareAndLoadParameter("traffic_light_opacity", traffic_light_opacity_, "Opacity of the traffic lights", true, false, false);

  this->declareAndLoadParameter("yield_sign_mesh_resource", yield_sign_mesh_resource_, "Link to the yield sign model to use", true, false, false);
  this->declareAndLoadParameter("yield_sign_scale", yield_sign_scale_, "Scale of the yield sign models", true, false, false);
  this->declareAndLoadParameter("yield_sign_z_offset", yield_sign_z_offset_, "Offset in z-direction of the yield sign models (depends on model and scale)", true, false, false);
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
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
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
    if constexpr(is_vector_v<T>) {
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
      if constexpr(is_vector_v<T>) {
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
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}


rcl_interfaces::msg::SetParametersResult Lanelet2LichtblickDisplay::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

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

  // Callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&Lanelet2LichtblickDisplay::parametersCallback, this, std::placeholders::_1));

  // Publisher for visualization markers
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(); 
  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lichtblick_lanelet2_map", qos);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s' with transient_local QoS", marker_array_publisher_->get_topic_name());

  ll2if_ = std::make_shared<LL2MapInterface>(*this, "ll2_map_server");

  // Periodically check for map updates
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
    
    unsigned int r_int, g_int, b_int;

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


geometry_msgs::msg::Point Lanelet2LichtblickDisplay::toRos(const Eigen::Vector3d &point) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = point.x();
  ros_point.y = point.y();
  ros_point.z = point.z();
  return ros_point;
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
  std::array<geometry_msgs::msg::Point, 2> reference_line_ros = {toRos(reference_line.front()),
                                                                 toRos(reference_line.back())};
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
      this->publishMarker(current_map_ptr);
      last_map_ptr_ = current_map_ptr;
      need_republish_.store(false); // cleared after publish
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

  float r, g, b;

  visualization_msgs::msg::MarkerArray marker_array_msg;
  int current_marker_id = 0;

  // Iterate through all lanelets in the map
  for (const auto& lanelet : lanelet_map->laneletLayer) {
    // --- Visualize Left Boundary ---
    visualization_msgs::msg::Marker left_bound_marker;
    left_bound_marker.header.frame_id = "map";
    left_bound_marker.header.stamp = this->now();
    left_bound_marker.ns = "lanelet_left_boundaries";
    left_bound_marker.id = current_marker_id++;
    left_bound_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    left_bound_marker.action = visualization_msgs::msg::Marker::ADD;
    left_bound_marker.scale.x = left_bound_line_width_;

    hexToRgb(left_bound_color_hex_, r, g, b);
    left_bound_marker.color.r = r;
    left_bound_marker.color.g = g;
    left_bound_marker.color.b = b;
    left_bound_marker.color.a = left_bound_line_opacity_;

    // Convert lanelet points to geometry_msgs::msg::Point
    for (const auto& p : lanelet.leftBound()) {
      left_bound_marker.points.push_back(toRos(p.basicPoint()));
    }
    marker_array_msg.markers.push_back(left_bound_marker);

    // --- Visualize Right Boundary ---
    visualization_msgs::msg::Marker right_bound_marker;
    right_bound_marker.header.frame_id = "map";
    right_bound_marker.header.stamp = this->now();
    right_bound_marker.ns = "lanelet_right_boundaries";
    right_bound_marker.id = current_marker_id++;
    right_bound_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    right_bound_marker.action = visualization_msgs::msg::Marker::ADD;
    right_bound_marker.scale.x = right_bound_line_width_;

    hexToRgb(right_bound_color_hex_, r, g, b);
    right_bound_marker.color.r = r;
    right_bound_marker.color.g = g;
    right_bound_marker.color.b = b;
    right_bound_marker.color.a = right_bound_line_opacity_;

    for (const auto& p : lanelet.rightBound()) {
      right_bound_marker.points.push_back(toRos(p.basicPoint()));
    }
    marker_array_msg.markers.push_back(right_bound_marker);

    // --- Visualize Centerline ---
    if (!lanelet.centerline().empty()) {
      visualization_msgs::msg::Marker centerline_marker;
      centerline_marker.header.frame_id = "map";
      centerline_marker.header.stamp = this->now();
      centerline_marker.ns = "lanelet_centerlines";
      centerline_marker.id = current_marker_id++;
      centerline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      centerline_marker.action = visualization_msgs::msg::Marker::ADD;
      centerline_marker.scale.x = centerline_line_width_;

      hexToRgb(centerline_color_hex_, r, g, b);
      centerline_marker.color.r = r;
      centerline_marker.color.g = g;
      centerline_marker.color.b = b;
      centerline_marker.color.a = centerline_line_opacity_;

      for (const auto& p : lanelet.centerline()) {
        centerline_marker.points.push_back(toRos(p.basicPoint()));
      }
      marker_array_msg.markers.push_back(centerline_marker);
    }

    const auto& regulatory_elements = lanelet.regulatoryElements();
    for (const auto& regulatory_element: regulatory_elements) {
      // Check if the element is a reference line
      if (auto reference_line = regulatoryElementReferenceLine(regulatory_element)) {
        // reference_line are LineStrings, so we can visualize them with a LINE_STRIP marker
        visualization_msgs::msg::Marker reference_line_marker;
        reference_line_marker.header.frame_id = "map";
        reference_line_marker.header.stamp = this->now();
        reference_line_marker.ns = "reference_lines";
        reference_line_marker.id = current_marker_id++;
        reference_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        reference_line_marker.action = visualization_msgs::msg::Marker::ADD;
        reference_line_marker.scale.x = reference_line_width_;

        hexToRgb(reference_line_color_hex_, r, g, b);
        reference_line_marker.color.r = r;
        reference_line_marker.color.g = g;
        reference_line_marker.color.b = b;
        reference_line_marker.color.a = reference_line_opacity_;

        // Extract and add the points
        reference_line_marker.points.push_back(reference_line->at(0));
        reference_line_marker.points.push_back(reference_line->at(1));

        marker_array_msg.markers.push_back(reference_line_marker);
      }

      // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/RegulatoryElementTagging.md
      std::string subtype = regulatory_element->attribute("subtype").value();
      if (subtype == "traffic_light" || subtype == "right_of_way") {
        // Get the positions of the traffic lights from the "refers" attribute
        std::vector<geometry_msgs::msg::Point> positions = regulatoryElementPositions(regulatory_element);

        // Retrieve the reference line points to calculate yaw
        if (auto ref_line_pts = regulatoryElementReferenceLine(regulatory_element)) {
          // Calculate yaw from the reference line direction
          double dx = ref_line_pts->at(1).x - ref_line_pts->at(0).x;
          double dy = ref_line_pts->at(1).y - ref_line_pts->at(0).y;
          double yaw_ref_line = std::atan2(dy, dx);

          geometry_msgs::msg::Point middle_point;
          middle_point.x = (ref_line_pts->at(0).x + ref_line_pts->at(1).x) / 2.0;
          middle_point.y = (ref_line_pts->at(0).y + ref_line_pts->at(1).y) / 2.0;

          // Create a marker for each position
          for (auto& p : positions) {
            if (subtype == "traffic_light") {
              visualization_msgs::msg::Marker tl_marker;
              tl_marker.header.frame_id = "map";
              tl_marker.header.stamp = this->now();
              tl_marker.ns = "traffic_lights";
              tl_marker.id = current_marker_id++;
              tl_marker.action = visualization_msgs::msg::Marker::ADD;
              tl_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
              tl_marker.mesh_resource = traffic_light_mesh_resource_;
              tl_marker.scale.x = traffic_light_scale_;
              tl_marker.scale.y = traffic_light_scale_;
              tl_marker.scale.z = traffic_light_scale_;

              // Set the position of the traffic sign (depends on scale and model)
              p.z = traffic_light_z_offset_;
              tl_marker.pose.position = p;

              // Set the orientation to be orthogonal to the reference line
              // Calculate the vector from traffic sign to the reference line
              double dx_to_ref = middle_point.x - p.x;
              double dy_to_ref = middle_point.y - p.y;
              double yaw_to_ref = std::atan2(dy_to_ref, dx_to_ref);

              // Two possible perpendicular orientations to the ref line:
              double cand1 = yaw_ref_line + M_PI_2; // +90°
              double cand2 = yaw_ref_line - M_PI_2; // -90°

              // measure angular difference (normalized) using atan2(sin,cos)
              double d1 = std::abs(std::atan2(std::sin(yaw_to_ref - cand1), std::cos(yaw_to_ref - cand1)));
              double d2 = std::abs(std::atan2(std::sin(yaw_to_ref - cand2), std::cos(yaw_to_ref - cand2)));

              double final_yaw = (d1 < d2) ? cand1 : cand2;

              tf2::Quaternion q;
              q.setRPY(0, 0, final_yaw + M_PI_2);
              tl_marker.pose.orientation = tf2::toMsg(q);

              tl_marker.mesh_use_embedded_materials = true;
              tl_marker.color.a = traffic_light_opacity_;

              marker_array_msg.markers.push_back(tl_marker);
            }
            if (subtype == "right_of_way") {
              visualization_msgs::msg::Marker yield_marker;
              yield_marker.header.frame_id = "map";
              yield_marker.header.stamp = this->now();
              yield_marker.ns = "yield_signs";
              yield_marker.id = current_marker_id++;
              yield_marker.action = visualization_msgs::msg::Marker::ADD;
              yield_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
              yield_marker.mesh_resource = yield_sign_mesh_resource_;
              yield_marker.scale.x = yield_sign_scale_;
              yield_marker.scale.y = yield_sign_scale_;
              yield_marker.scale.z = yield_sign_scale_;

              // Set the position of the traffic sign (depends on scale and model)
              p.z = yield_sign_z_offset_;
              yield_marker.pose.position = p;

              // Set the orientation to be orthogonal to the reference line
              // Calculate the vector from traffic sign to the reference line
              double dx_to_ref = middle_point.x - p.x;
              double dy_to_ref = middle_point.y - p.y;
              double yaw_to_ref = std::atan2(dy_to_ref, dx_to_ref);

              // Two possible perpendicular orientations to the ref line:
              double cand1 = yaw_ref_line + M_PI_2; // +90°
              double cand2 = yaw_ref_line - M_PI_2; // -90°

              // measure angular difference (normalized) using atan2(sin,cos)
              double d1 = std::abs(std::atan2(std::sin(yaw_to_ref - cand1), std::cos(yaw_to_ref - cand1)));
              double d2 = std::abs(std::atan2(std::sin(yaw_to_ref - cand2), std::cos(yaw_to_ref - cand2)));

              double final_yaw = (d1 < d2) ? cand1 : cand2;

              tf2::Quaternion q;
              q.setRPY(0, 0, final_yaw + M_PI_2);
              yield_marker.pose.orientation = tf2::toMsg(q);

              yield_marker.mesh_use_embedded_materials = true;
              yield_marker.color.a = yield_sign_opacity_;

              marker_array_msg.markers.push_back(yield_marker);
            }
          }
        }
      }
    }
  }

  // Set the lifetime for all markers in the array to indefinite since the QoS is set to transient local.
  for (auto& marker : marker_array_msg.markers) {
      marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(0));
  }

  // Publish the marker array
  marker_array_publisher_->publish(marker_array_msg);
}

} // namespace lanelet2_lichtblick_display


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lanelet2_lichtblick_display::Lanelet2LichtblickDisplay>());
  rclcpp::shutdown();

  return 0;
}
