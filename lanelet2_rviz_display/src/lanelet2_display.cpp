// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include "lanelet2_rviz_display/lanelet2_display.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/properties/property.hpp"

using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::qtToOgre;
using rviz_common::properties::StringProperty;

namespace lanelet2_rviz_display {

Lanelet2Display::Lanelet2Display() {
  ll2_server_name_property_ =
      new StringProperty(QString::fromStdString("Lanelet2-Map-Server Name"), QString::fromStdString("ll2_map_server"),
                         QString::fromStdString("Global name of the Lanelet2-Map-Server."), this, SLOT(updateServerName()));

  alpha_property_ =
      new FloatProperty("Alpha", 1.0f, "The amount of transparency to apply to the Map.", this, SLOT(updateColor()), this);
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  three_d_property_ =
      new BoolProperty("Show Map in 3D", true, "Toggles wether to display the lanelet map with or without its z coordinates.",
                       this, SLOT(update3D()));

  viz_linestring_property_ =
      new BoolProperty("Visualize Lanelet-Linestrings", true, "Activate the visualization of Lanelet-Linestrings.", this,
                       SLOT(updateLinestringRendering()));

  ll_left_col_property_ = new ColorProperty("LL Color Left", Qt::white, "Color of left Lanelet-Linestring.",
                                            viz_linestring_property_, SLOT(updateColor()), this);

  ll_right_col_property_ = new ColorProperty("LL Color Right", Qt::white, "Color of right Lanelet-Linestring.",
                                             viz_linestring_property_, SLOT(updateColor()), this);

  linestring_width_property_ = new FloatProperty("Linestring Width", 0.1f, "The width, in meters, of each linestring.",
                                                 viz_linestring_property_, SLOT(updateStyle()), this);
  linestring_width_property_->setMin(0.01f);

  viz_separators_property_ =
      new BoolProperty("Visualize Lanelet-Separators", false, "Activate the visualization of Lanelet-Separators.", this,
                       SLOT(updateSeparatorsRendering()));

  separators_col_property_ = new ColorProperty("Separators Color", QColor{25, 25, 230}, "Color of Lanelet Separators.",
                                               viz_separators_property_, SLOT(updateColor()), this);

  separators_width_property_ = new FloatProperty("Linestring Width", 0.2f, "The width, in meters, of each linestring.",
                                                 viz_separators_property_, SLOT(updateStyle()), this);
  separators_width_property_->setMin(0.01f);

  viz_stop_line_property_ = new BoolProperty("Visualize Stop lines", true, "Activate the visualization of Stop-Lines.", this,
                                             SLOT(updateStopLineRendering()));

  stop_line_col_property_ = new ColorProperty("Stop Line Color", QColor{255, 25, 25}, "Color of Stop-Lines.",
                                              viz_stop_line_property_, SLOT(updateColor()), this);

  stop_line_width_property_ = new FloatProperty("Stop Line Width", 0.2f, "The width, in meters, of each stop line.",
                                                viz_stop_line_property_, SLOT(updateStyle()), this);
  stop_line_width_property_->setMin(0.01f);

  viz_traffic_light_property_ =
      new BoolProperty("Visualize Traffic Lights", true, "Activate the visualization of Traffic-Lights.", this,
                       SLOT(updateTrafficLightRendering()));

  traffic_light_col_property_ = new ColorProperty("Traffic Light Color", QColor{102, 102, 102}, "Color of Traffic-Lights.",
                                                  viz_traffic_light_property_, SLOT(updateColor()), this);

  traffic_light_height_property_ =
      new FloatProperty("Traffic Light Height", 3.0f, "The height, in meters, of each traffic light.",
                        viz_traffic_light_property_, SLOT(updateStyle()), this);
  traffic_light_height_property_->setMin(0.01f);

  viz_area_property_ =
      new BoolProperty("Visualize Areas", true, "Activate the visualization of Areas.", this, SLOT(updateAreaRendering()));

  area_col_property_ =
      new ColorProperty("Area Color", QColor{230, 127, 25}, "Color of Areas.", viz_area_property_, SLOT(updateColor()), this);

  area_width_property_ =
      new FloatProperty("Area Width", 0.3f, "The width, in meters, of each area.", viz_area_property_, SLOT(updateStyle()), this);
  area_width_property_->setMin(0.01f);

  fill_area_property_ = new BoolProperty("Fill Areas", false, "Toggles wether to fill the areas with color or not.",
                                         viz_area_property_, SLOT(updateStyle()), this);

  viz_parking_property_ =
      new BoolProperty("Visualize Parking", true, "Activate the visualization of Parking.", this, SLOT(updateParkingRendering()));

  parking_col_property_ = new ColorProperty("Parking Color", QColor{0, 179, 76}, "Color of Parking.", viz_parking_property_,
                                            SLOT(updateColor()), this);

  parking_width_property_ = new FloatProperty("Parking Width", 0.3f, "The width, in meters, of each parking.",
                                              viz_parking_property_, SLOT(updateStyle()), this);
  parking_width_property_->setMin(0.01f);

  fill_parking_property_ = new BoolProperty("Fill Parking", true, "Toggles wether to fill the parking with color or not.",
                                            viz_parking_property_, SLOT(updateStyle()), this);

  // Lane fills (road surface)
  viz_lane_fill_property_ = new BoolProperty("Fill Lanelets (Road)", true, "Render road surface as filled polygons.", this,
                                             SLOT(updateLaneFillRendering()));
  lane_fill_col_property_ = new ColorProperty("Road Color", QColor{56, 56, 56}, "Color of road fill.", viz_lane_fill_property_,
                                              SLOT(updateColor()), this);

  // Pedestrian features
  viz_sidewalk_property_ = new BoolProperty("Visualize Sidewalks", true, "Render sidewalk areas as filled polygons.", this,
                                            SLOT(updateSidewalkRendering()));
  sidewalk_col_property_ = new ColorProperty("Sidewalk Color", QColor{191, 191, 191}, "Color of sidewalks.",
                                             viz_sidewalk_property_, SLOT(updateColor()), this);

  viz_crosswalk_property_ = new BoolProperty("Visualize Crosswalks", true, "Render crosswalk areas as filled polygons.", this,
                                             SLOT(updateCrosswalkRendering()));
  crosswalk_col_property_ = new ColorProperty("Crosswalk Color", QColor{255, 255, 255}, "Color of crosswalks.",
                                              viz_crosswalk_property_, SLOT(updateColor()), this);

  viz_id_property_ = new BoolProperty("Visualize Lanelet-IDs", false, "Activate the visualization of Lanelet-IDs.", this,
                                      SLOT(updateIdRendering()));

  id_col_property_ =
      new ColorProperty("ID Color", Qt::white, "Color of Lanelet-IDs.", viz_id_property_, SLOT(updateColor()), this);

  char_height_property_ =
      new FloatProperty("Character Height", 1.0f, "The height of each character.", viz_id_property_, SLOT(updateStyle()), this);
}

Lanelet2Display::~Lanelet2Display() = default;

void Lanelet2Display::initializeMapInterface(rclcpp::Node& parent_node) {
  std::string name = ll2_server_name_property_->getStdString();
  if (!name.empty()) {
    if (name[0] != '/') {
      name = '/' + name;
    }
    ll2if_ = new LL2MapInterface(parent_node, name);
  }
}

void Lanelet2Display::onInitialize() {
  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();
  initializeMapInterface(*rviz_node_);
}

void Lanelet2Display::update(float dt, float ros_dt) {
  if (ll2if_->map_loaded_) {
    if (!viz_init_) {
      viz_init_ = visualizeMap();
      ll2if_->update_pending_ = false;
    }

    if (ll2if_->update_pending_) {
      updateVisualization();
    }

    if (viz_init_) {
      Q_UNUSED(dt);
      Q_UNUSED(ros_dt);

      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      if (context_->getFrameManager()->getTransform(ll2if_->map_frame_id_, position, orientation)) {
        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);
        setTransformOk();
      } else {
        setMissingTransformToFixedFrame(ll2if_->map_frame_id_);
        map_->getSceneNode()->setVisible(false);
      }
    }
  }
}

void Lanelet2Display::updateServerName() {
  delete ll2if_;
  initializeMapInterface(*rviz_node_);
  viz_init_ = false;
}

bool Lanelet2Display::visualizeMap() {
  map_ = std::make_unique<rviz_rendering::Lanelet2Map>(scene_manager_, scene_node_, rendering_options_, ll2if_->getMapPtr());
  return true;
}

void Lanelet2Display::updateVisualization() {
  if (viz_init_) {
    map_->updateMap(rendering_options_, ll2if_->getMapPtr());
    ll2if_->update_pending_ = false;
  }
}

void Lanelet2Display::updateVisibility() {
  if (viz_init_) {
    map_->updateVisibility(rendering_options_);
  }
}

void Lanelet2Display::updateLinestringRendering() {
  rendering_options_.renderLaneletLinestrings = viz_linestring_property_->getBool();
  for (uint16_t i = 0; i < viz_linestring_property_->numChildren(); i++) {
    viz_linestring_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderLaneletLinestrings);
  }
  updateVisibility();
}

void Lanelet2Display::updateSeparatorsRendering() {
  rendering_options_.renderLaneletSeparators = viz_separators_property_->getBool();
  for (uint16_t i = 0; i < viz_separators_property_->numChildren(); i++) {
    viz_separators_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderLaneletSeparators);
  }
  updateVisibility();
}

void Lanelet2Display::updateStopLineRendering() {
  rendering_options_.renderStopLines = viz_stop_line_property_->getBool();
  for (uint16_t i = 0; i < viz_stop_line_property_->numChildren(); i++) {
    viz_stop_line_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderStopLines);
  }
  updateVisibility();
}

void Lanelet2Display::updateTrafficLightRendering() {
  rendering_options_.renderTrafficLights = viz_traffic_light_property_->getBool();
  for (uint16_t i = 0; i < viz_traffic_light_property_->numChildren(); i++) {
    viz_traffic_light_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderTrafficLights);
  }
  updateVisibility();
}

void Lanelet2Display::updateAreaRendering() {
  rendering_options_.renderAreas = viz_area_property_->getBool();
  for (uint16_t i = 0; i < viz_area_property_->numChildren(); i++) {
    viz_area_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderAreas);
  }
  updateVisibility();
}

void Lanelet2Display::updateParkingRendering() {
  rendering_options_.renderParking = viz_parking_property_->getBool();
  for (uint16_t i = 0; i < viz_parking_property_->numChildren(); i++) {
    viz_parking_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderParking);
  }
  updateVisibility();
}

void Lanelet2Display::updateLaneFillRendering() {
  rendering_options_.renderLaneletFills = viz_lane_fill_property_->getBool();
  for (uint16_t i = 0; i < viz_lane_fill_property_->numChildren(); i++) {
    viz_lane_fill_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderLaneletFills);
  }
  updateVisibility();
}

void Lanelet2Display::updateSidewalkRendering() {
  rendering_options_.renderSidewalks = viz_sidewalk_property_->getBool();
  for (uint16_t i = 0; i < viz_sidewalk_property_->numChildren(); i++) {
    viz_sidewalk_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderSidewalks);
  }
  updateVisibility();
}

void Lanelet2Display::updateCrosswalkRendering() {
  rendering_options_.renderCrosswalks = viz_crosswalk_property_->getBool();
  for (uint16_t i = 0; i < viz_crosswalk_property_->numChildren(); i++) {
    viz_crosswalk_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderCrosswalks);
  }
  updateVisibility();
}

void Lanelet2Display::updateIdRendering() {
  rendering_options_.renderLaneletIds = viz_id_property_->getBool();
  for (uint16_t i = 0; i < viz_id_property_->numChildren(); i++) {
    viz_id_property_->childAtUnchecked(i)->setHidden(!rendering_options_.renderLaneletIds);
  }
  updateVisibility();
}

void Lanelet2Display::updateColor() {
  // Linestrings
  QColor color_left = ll_left_col_property_->getColor();
  QColor color_right = ll_right_col_property_->getColor();
  color_left.setAlphaF(alpha_property_->getFloat());
  color_right.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorLeft = qtToOgre(color_left);
  rendering_options_.colorRight = qtToOgre(color_right);

  // Separators
  QColor color_sep = separators_col_property_->getColor();
  color_sep.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorSeperator = qtToOgre(color_sep);

  // Traffic lights
  QColor color_tl = traffic_light_col_property_->getColor();
  color_tl.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorTrafficLight = qtToOgre(color_tl);

  // Stop lines
  QColor color_sl = stop_line_col_property_->getColor();
  color_sl.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorStopLine = qtToOgre(color_sl);

  // Areas
  QColor color_area = area_col_property_->getColor();
  color_area.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorArea = qtToOgre(color_area);

  // Parking
  QColor color_parking = parking_col_property_->getColor();
  color_parking.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorParking = qtToOgre(color_parking);

  // Lane fill (road)
  QColor color_lane_fill = lane_fill_col_property_->getColor();
  color_lane_fill.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorLaneFill = qtToOgre(color_lane_fill);

  // Sidewalk
  QColor color_sidewalk = sidewalk_col_property_->getColor();
  color_sidewalk.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorSidewalk = qtToOgre(color_sidewalk);

  // Crosswalk
  QColor color_crosswalk = crosswalk_col_property_->getColor();
  color_crosswalk.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorCrosswalk = qtToOgre(color_crosswalk);

  // IDs
  QColor color_id = id_col_property_->getColor();
  color_id.setAlphaF(alpha_property_->getFloat());
  rendering_options_.colorLaneletId = qtToOgre(color_id);

  updateVisualization();
}

void Lanelet2Display::updateStyle() {
  // Linestrings
  rendering_options_.linestringWidth = linestring_width_property_->getFloat();

  // Separators
  rendering_options_.seperatorWidth = separators_width_property_->getFloat();

  // Stop lines
  rendering_options_.stopLineWidth = stop_line_width_property_->getFloat();

  // Traffic lights
  rendering_options_.trafficLightHeightAboveGround = traffic_light_height_property_->getFloat();

  // IDs
  rendering_options_.characterHeight = char_height_property_->getFloat();

  // Areas
  rendering_options_.fillArea = fill_area_property_->getBool();

  // Parking
  rendering_options_.fillParking = fill_parking_property_->getBool();

  updateVisualization();
}

void Lanelet2Display::update3D() {
  // 3D
  rendering_options_.threeD = three_d_property_->getBool();

  updateVisualization();
}

}  // namespace lanelet2_rviz_display

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(lanelet2_rviz_display::Lanelet2Display, rviz_common::Display)
