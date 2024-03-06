#include "lanelet2_rviz_display/lanelet2_display.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/interaction/selection_manager.hpp"

using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::qtToOgre;
using rviz_common::properties::StringProperty;

namespace lanelet2_rviz_display
{

Lanelet2Display::Lanelet2Display()
{
  ll2_server_name_property_ = new StringProperty(
    QString::fromStdString("Lanelet2-Map-Server Name"),
    QString::fromStdString("ll2_map_server"),
    QString::fromStdString("Name of the Lanelet2-Map-Server."),
    this, SLOT(updateServerName()));

  alpha_property_ = new FloatProperty(
    "Alpha", 1.0f,
    "The amount of transparency to apply to the Map.",
    this, SLOT(updateColor()), this);
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  viz_linestring_property_ = new BoolProperty(
    "Visualize Lanelet-Linestrings", true,
    "Activate the visualization of Lanelet-Linestrings.",
    this, SLOT(updateLinestringRendering()));

  ll_left_col_property_ = new ColorProperty(
    "LL Color Left", Qt::white,
    "Color of left Lanelet-Linestring.",
    viz_linestring_property_, SLOT(updateColor()), this);

  ll_right_col_property_ = new ColorProperty(
    "LL Color Right", Qt::white,
    "Color of right Lanelet-Linestring.",
    viz_linestring_property_, SLOT(updateColor()), this);

  linestring_width_property_ = new FloatProperty(
    "Linestring Width", 0.1f,
    "The width, in meters, of each linestring.",
    viz_linestring_property_, SLOT(updateWidth()), this);
  linestring_width_property_->setMin(0.01f);

  viz_separators_property_ = new BoolProperty(
    "Visualize Lanelet-Separators", false,
    "Activate the visualization of Lanelet-Separators.",
    this, SLOT(updateSeparatorsRendering()));

  separators_col_property_ = new ColorProperty(
    "Separators Color", Qt::blue,
    "Color of Lanelet Separators.",
    viz_separators_property_, SLOT(updateColor()), this);

  separators_width_property_ = new FloatProperty(
    "Linestring Width", 0.1f,
    "The width, in meters, of each linestring.",
    viz_separators_property_, SLOT(updateWidth()), this);
  separators_width_property_->setMin(0.01f);
  
  three_d_property_ = new BoolProperty(
    "Show Map in 3D", true,
    "Toggles wether to display the lanelet map with or without its z coordinates.",
    this, SLOT(update3D()));

}

Lanelet2Display::~Lanelet2Display() = default;

void Lanelet2Display::initializeMapInterface(rclcpp::Node& parent_node)
{
  std::string name = ll2_server_name_property_->getStdString();
  ll2if_ = new LL2MapInterface(parent_node, name);
}

void Lanelet2Display::onInitialize()
{
  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();
  initializeMapInterface(*rviz_node_);
}

void Lanelet2Display::update(float dt, float ros_dt)
{
  if(ll2if_->map_loaded_)
  {
    if(!viz_init_ )
    {
      viz_init_ = visualizeMap();
      ll2if_->update_pending_=false;
    }

    if(ll2if_->update_pending_)
    {
      updateVisualization();
    }
    
    if(viz_init_)
    {
      Q_UNUSED(dt);
      Q_UNUSED(ros_dt);

      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      if (context_->getFrameManager()->getTransform(ll2if_->map_frame_id_, position, orientation)) {
        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);
        setTransformOk();
        map_->getSceneNode()->setVisible(true);
      } else {
        setMissingTransformToFixedFrame(ll2if_->map_frame_id_);
        map_->getSceneNode()->setVisible(false);
      }
    }
  }
}

void Lanelet2Display::updateServerName()
{
  delete ll2if_;
  initializeMapInterface(rviz_node_);
  viz_init_=false;
}

bool Lanelet2Display::visualizeMap()
{
  map_ = std::make_unique<rviz_rendering::Lanelet2Map>(scene_manager_, scene_node_, rendering_options_, ll2if_->getMapPtr());
  map_->getSceneNode()->setVisible(false);
  return true;
}

void Lanelet2Display::updateVisualization()
{
  if(viz_init_)
  {
    map_->updateMap(rendering_options_, ll2if_->getMapPtr());
    ll2if_->update_pending_=false;
  }
}

void Lanelet2Display::updateLinestringRendering()
{
  rendering_options_.renderLaneletLinestrings = viz_linestring_property_->getBool();
  if(!rendering_options_.renderLaneletLinestrings)
  {
    viz_linestring_property_->collapse();
  }
  else
  {
    viz_linestring_property_->expand();
  }
  updateVisualization();
}

void Lanelet2Display::updateSeparatorsRendering()
{
  rendering_options_.renderLaneletSeparators = viz_separators_property_->getBool();
  if(!rendering_options_.renderLaneletSeparators)
  {
    viz_separators_property_->collapse();
  }
  else
  {
    viz_separators_property_->expand();
  }
  updateVisualization();
}

void Lanelet2Display::updateColor()
{
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

  updateVisualization();
}

void Lanelet2Display::updateWidth()
{
  // Linestrings
  rendering_options_.linestringWidth = linestring_width_property_->getFloat();

  // Separators
  rendering_options_.seperatorWidth = separators_width_property_->getFloat();

  updateVisualization();
}

void Lanelet2Display::update3D()
{
  // 3D
  rendering_options_.threeD = three_d_property_->getBool();

  updateVisualization();
}

} // namespace lanelet2_rviz_display

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(lanelet2_rviz_display::Lanelet2Display, rviz_common::Display)