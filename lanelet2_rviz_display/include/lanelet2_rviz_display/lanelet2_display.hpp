#ifndef LANELET2_DISPLAY_HPP_
#define LANELET2_DISPLAY_HPP_

#include <memory>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/bool_property.hpp"

#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
#include "lanelet2_rviz_display/lanelet2_map.hpp"


/**
 * \class Lanelet2 Display
 * \brief Displays a Lanelet2 Map.
 *
 */
namespace lanelet2_rviz_display
{

class Lanelet2Display : public rviz_common::Display
{
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
  void updateColor();
  void updateWidth();
  void update3D();

private:

  void initializeMapInterface(rclcpp::Node::SharedPtr parent_node);
  bool visualizeMap();
  void updateVisualization();


  LL2MapInterface *ll2if_;
  rclcpp::Node::SharedPtr rviz_node_;

  bool viz_init_=false;

  std::unique_ptr<rviz_rendering::Lanelet2Map> map_;  // Handles actually drawing the ll2-map
  rviz_rendering::Lanelet2Map::RenderingOptions rendering_options_;

  rviz_common::properties::StringProperty *ll2_server_name_property_;
  rviz_common::properties::BoolProperty *viz_linestring_property_, *viz_separators_property_, *three_d_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
  rviz_common::properties::ColorProperty *ll_left_col_property_, *ll_right_col_property_, *separators_col_property_;
  rviz_common::properties::FloatProperty *linestring_width_property_, *separators_width_property_;

};

} //namespace lanelet2_rviz_display

#endif  // LANELET2_DISPLAY_HPP_
