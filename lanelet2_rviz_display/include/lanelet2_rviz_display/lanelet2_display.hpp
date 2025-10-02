#ifndef LANELET2_DISPLAY_HPP_
#define LANELET2_DISPLAY_HPP_

#include <memory>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/string_property.hpp"

#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
#include "lanelet2_rviz_display/lanelet2_map.hpp"

/**
 * \class Lanelet2 Display
 * \brief Displays a Lanelet2 Map.
 *
 */
namespace lanelet2_rviz_display {

class Lanelet2Display : public rviz_common::Display {
  Q_OBJECT

 public:
  Lanelet2Display();
  ~Lanelet2Display() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float dt, float ros_dt) override;

 private Q_SLOTS:

  void updateServerName();

  void updateLinestringRendering();
  void updateSeparatorsRendering();
  void updateStopLineRendering();
  void updateTrafficLightRendering();
  void updateAreaRendering();
  void updateParkingRendering();
  void updateLaneFillRendering();
  void updateSidewalkRendering();
  void updateCrosswalkRendering();
  void updateIdRendering();
  void updateColor();
  void updateStyle();
  void update3D();

 private:
  void initializeMapInterface(rclcpp::Node &parent_node);
  bool visualizeMap();
  void updateVisualization();
  void updateVisibility();

  LL2MapInterface *ll2if_;
  rclcpp::Node::SharedPtr rviz_node_;

  bool viz_init_ = false;

  std::unique_ptr<rviz_rendering::Lanelet2Map> map_;  // Handles actually drawing the ll2-map
  rviz_rendering::Lanelet2Map::RenderingOptions rendering_options_;

  rviz_common::properties::StringProperty *ll2_server_name_property_;
  rviz_common::properties::BoolProperty *viz_linestring_property_, *viz_separators_property_, *three_d_property_,
      *fill_area_property_, *fill_parking_property_, *viz_stop_line_property_, *viz_traffic_light_property_,
      *viz_area_property_, *viz_parking_property_, *viz_id_property_, *viz_lane_fill_property_, *viz_sidewalk_property_,
      *viz_crosswalk_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
  rviz_common::properties::ColorProperty *ll_left_col_property_, *ll_right_col_property_, *separators_col_property_,
      *traffic_light_col_property_, *stop_line_col_property_, *area_col_property_, *parking_col_property_,
      *id_col_property_, *lane_fill_col_property_, *sidewalk_col_property_, *crosswalk_col_property_;
  rviz_common::properties::FloatProperty *linestring_width_property_, *separators_width_property_,
      *stop_line_width_property_, *traffic_light_height_property_, *area_width_property_, *parking_width_property_,
      *char_height_property_;
};

}  //namespace lanelet2_rviz_display

#endif  // LANELET2_DISPLAY_HPP_
