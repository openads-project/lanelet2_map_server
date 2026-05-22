// Copyright (c) 2017, FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
// and KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: BSD-3-Clause

/*
 * Derived in part from:
 * https://github.com/coincar-sim/lanelet_rviz_plugin_ros/tree/9a36cb891ef0d175e0a893526980ea515712e1b9
 * - src/map_element.cpp
 * - src/map_element_ogre_helper.hpp
 */

#include "lanelet2_map_helpers.hpp"

#include <OgreMovableObject.h>
#include <OgreSceneManager.h>

#include <algorithm>
#include <string>

namespace rviz_rendering::detail {

void Lanelet2MapHelpers::clearObjects() {
  for (auto object : owner_.objects_) {
    switch (object.first) {
      case ObjectClassification::MAP:
      case ObjectClassification::AREA:
      case ObjectClassification::PARKINGAREA:
      case ObjectClassification::SEPERATOR:
      case ObjectClassification::STOPLINE:
      case ObjectClassification::TRAFFICLIGHT:
      case ObjectClassification::LANEFILL:
      case ObjectClassification::SIDEWALK:
      case ObjectClassification::CROSSWALK: {
        auto man_object = dynamic_cast<Ogre::ManualObject*>(object.second);
        if (man_object) {
          man_object->detachFromParent();
          owner_.scene_manager_->destroyManualObject(man_object);
        }
        break;
      }

      case ObjectClassification::LANELETID: {
        auto mov_text = dynamic_cast<rviz_rendering::MovableText*>(object.second);
        if (mov_text) {
          mov_text->detachFromParent();
          // MovableText is created directly and attached to the scene node, so it is destroyed directly here.
          // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
          delete mov_text;
        }
        break;
      }

      default: {
        if (object.second) {
          object.second->detachFromParent();
          // Fallback for directly owned OGRE objects not handled by the scene manager helpers above.
          // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
          delete object.second;
        }
      }
    }
  }

  Lanelet2Map::manual_object_counter_ = 0;
  owner_.objects_.clear();
}

void Lanelet2MapHelpers::addLaneletToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual) {
  auto left_bound = lanelet.leftBound();
  auto line_left = ogreLineFromLLetLineString(left_bound);
  drawLine(line_left, manual, owner_.rend_opts_.colorLeft, owner_.rend_opts_.linestringWidth, owner_.rend_opts_.zRoadLines);

  auto right_bound = lanelet.rightBound();
  auto line_right = ogreLineFromLLetLineString(right_bound);
  drawLine(line_right, manual, owner_.rend_opts_.colorRight, owner_.rend_opts_.linestringWidth, owner_.rend_opts_.zRoadLines);
}

void Lanelet2MapHelpers::addSeperatorToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual) {
  lanelet::ConstPoints3d first_points;
  first_points.push_back(lanelet.leftBound().front());
  first_points.push_back(lanelet.rightBound().front());
  auto line = ogreLineFromLLetPts(first_points);
  drawLine(line, manual, owner_.rend_opts_.colorSeperator, owner_.rend_opts_.seperatorWidth, owner_.rend_opts_.zSeparator);

  lanelet::ConstPoints3d last_points;
  last_points.push_back(lanelet.leftBound().back());
  last_points.push_back(lanelet.rightBound().back());
  line = ogreLineFromLLetPts(last_points);
  drawLine(line, manual, owner_.rend_opts_.colorSeperator, owner_.rend_opts_.seperatorWidth, owner_.rend_opts_.zSeparator);
}

void Lanelet2MapHelpers::addAreaToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual) {
  const lanelet::CompoundPolygon3d outer_polygon = area.outerBoundPolygon();
  const auto area_line = ogreLineFromLLetPolygon(outer_polygon);
  if (owner_.rend_opts_.fillArea) {
    drawArea(area_line, manual, owner_.rend_opts_.colorArea, owner_.rend_opts_.zAreas);
  } else {
    drawLine(area_line, manual, owner_.rend_opts_.colorArea, owner_.rend_opts_.areaWidth, owner_.rend_opts_.zAreas);
  }
}

void Lanelet2MapHelpers::addParkingAreaToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual) {
  const lanelet::CompoundPolygon3d outer_polygon = area.outerBoundPolygon();
  const auto area_line = ogreLineFromLLetPolygon(outer_polygon);
  if (owner_.rend_opts_.fillParking) {
    drawArea(area_line, manual, owner_.rend_opts_.colorParking, owner_.rend_opts_.zParking);
  } else {
    drawLine(area_line, manual, owner_.rend_opts_.colorParking, owner_.rend_opts_.parkingWidth, owner_.rend_opts_.zParking);
  }
}

void Lanelet2MapHelpers::addLaneFillToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual) {
  const auto left = ogreLineFromLLetLineString(lanelet.leftBound());
  const auto right = ogreLineFromLLetLineString(lanelet.rightBound());
  drawLaneFillStrip(left, right, manual, owner_.rend_opts_.colorLaneFill, owner_.rend_opts_.zLaneFill);
}

void Lanelet2MapHelpers::addSidewalkToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual) {
  const auto area_line = ogreLineFromLLetPolygon(area.outerBoundPolygon());
  drawArea(area_line, manual, owner_.rend_opts_.colorSidewalk, owner_.rend_opts_.zSidewalk);
}

void Lanelet2MapHelpers::addCrosswalkToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual) {
  const auto area_line = ogreLineFromLLetPolygon(area.outerBoundPolygon());
  drawArea(area_line, manual, owner_.rend_opts_.colorCrosswalk, owner_.rend_opts_.zCrosswalk);
}

void Lanelet2MapHelpers::addRegulatoryElements(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parent_node) {
  Ogre::SceneNode* regulatory_elements_node = parent_node->createChildSceneNode();
  auto regulatory_elements = lanelet.regulatoryElements();
  auto traffic_light_regelems = lanelet.regulatoryElementsAs<lanelet::TrafficLight>();

  if (owner_.rend_opts_.renderStopLines) {
    for (auto&& reg_element : regulatory_elements) {
      auto ref_lines = reg_element.get()->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
      attachRefLinesToSceneNode(ref_lines, regulatory_elements_node);
    }
  }

  if (owner_.rend_opts_.renderTrafficLights) {
    for (auto&& traffic_light : traffic_light_regelems) {
      auto traffic_lights = traffic_light.get()->getParameters<lanelet::ConstPolygon3d>(lanelet::RoleName::Refers);
      attachTrafficLightsToSceneNode(traffic_lights, regulatory_elements_node);
    }
  }
}

void Lanelet2MapHelpers::attachRefLinesToSceneNode(std::vector<lanelet::ConstLineString3d>& stop_lines,
                                                   Ogre::SceneNode* parent_node) {
  Ogre::ManualObject* stop_lines_manual_object =
      owner_.scene_manager_->createManualObject("llet_object_" + std::to_string(Lanelet2Map::manual_object_counter_++));
  stop_lines_manual_object->begin(owner_.material_line_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  stop_lines_manual_object->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN + 6);
  for (auto&& stop_line : stop_lines) {
    auto line = ogreLineFromLLetLineString(stop_line);
    drawLine(line, stop_lines_manual_object, owner_.rend_opts_.colorStopLine, owner_.rend_opts_.stopLineWidth,
             owner_.rend_opts_.zStopLine);
  }
  // OGRE 1.x exposes section counts through this deprecated API; RViz still uses this vendor version.
  // NOLINTNEXTLINE(clang-diagnostic-deprecated-declarations)
  if (stop_lines_manual_object->getNumSections()) {
    parent_node->attachObject(stop_lines_manual_object);
    owner_.objects_.push_back(std::make_pair(ObjectClassification::STOPLINE, stop_lines_manual_object));
  }
  stop_lines_manual_object->end();
}

void Lanelet2MapHelpers::attachTrafficLightsToSceneNode(std::vector<lanelet::ConstPolygon3d>& traffic_lights,
                                                        Ogre::SceneNode* parent_node) {
  Ogre::ManualObject* traffic_light_manual_object =
      owner_.scene_manager_->createManualObject("llet_object_" + std::to_string(Lanelet2Map::manual_object_counter_++));
  traffic_light_manual_object->begin(owner_.material_surface_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST,
                                     "rviz_rendering");
  traffic_light_manual_object->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN + 4);
  for (auto&& traffic_light : traffic_lights) {
    auto line = ogreLineFromLLetTrafficLight(traffic_light);
    drawArea(line, traffic_light_manual_object, owner_.rend_opts_.colorTrafficLight, owner_.rend_opts_.zStopLine);
  }
  // OGRE 1.x exposes section counts through this deprecated API; RViz still uses this vendor version.
  // NOLINTNEXTLINE(clang-diagnostic-deprecated-declarations)
  if (traffic_light_manual_object->getNumSections()) {
    parent_node->attachObject(traffic_light_manual_object);
    owner_.objects_.push_back(std::make_pair(ObjectClassification::TRAFFICLIGHT, traffic_light_manual_object));
  }
  traffic_light_manual_object->end();
}

void Lanelet2MapHelpers::attachLaneletIdToSceneNode(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parent_node) {
  Ogre::SceneNode* child_node = parent_node->createChildSceneNode();

  auto* msg = new rviz_rendering::MovableText(std::to_string(lanelet.id()));
  msg->setCharacterHeight(boost::numeric_cast<Ogre::Real>(owner_.rend_opts_.characterHeight));
  msg->setColor(owner_.rend_opts_.colorLaneletId);
  msg->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);

  lanelet::ConstPoint3d text_pos(
      lanelet::utils::getId(), lanelet.centerline()[lanelet.centerline().size() / 2].x(),
      lanelet.centerline()[lanelet.centerline().size() / 2].y(),
      owner_.rend_opts_.threeD ? lanelet.centerline()[lanelet.centerline().size() / 2].z() - 0.2 : -0.2);

  child_node->setPosition(ogreVec3FromLLetPoint(text_pos));
  child_node->attachObject(msg);
  owner_.objects_.push_back(std::make_pair(ObjectClassification::LANELETID, msg));
}

std::vector<Ogre::Vector3> Lanelet2MapHelpers::ogreLineFromLLetLineString(const lanelet::ConstLineString3d& line_string) const {
  std::vector<Ogre::Vector3> line;
  for (lanelet::ConstPoint3d point : line_string) {
    line.push_back(ogreVec3FromLLetPoint(point));
  }
  return line;
}

std::vector<Ogre::Vector3> Lanelet2MapHelpers::ogreLineFromLLetPolygon(const lanelet::CompoundPolygon3d& polygon) const {
  std::vector<Ogre::Vector3> line;
  for (lanelet::ConstPoint3d point : polygon) {
    line.push_back(ogreVec3FromLLetPoint(point));
  }
  return line;
}

std::vector<Ogre::Vector3> Lanelet2MapHelpers::ogreLineFromLLetTrafficLight(const lanelet::ConstPolygon3d& polygon) const {
  if (polygon.empty()) {
    return {};
  }

  std::vector<Ogre::Vector3> line;
  const lanelet::ConstPoint3d lowest_point =
      *std::min_element(polygon.begin(), polygon.end(),
                        [](const lanelet::ConstPoint3d& p1, const lanelet::ConstPoint3d& p2) { return p1.z() < p2.z(); });

  for (lanelet::ConstPoint3d point : polygon) {
    line.push_back(ogreVec3FromLLetTrafficLight(point, owner_.rend_opts_.trafficLightHeightAboveGround - lowest_point.z()));
  }
  return line;
}

std::vector<Ogre::Vector3> Lanelet2MapHelpers::ogreLineFromLLetPts(const lanelet::ConstPoints3d& pts_vector) const {
  std::vector<Ogre::Vector3> line;
  for (lanelet::ConstPoint3d point : pts_vector) {
    line.push_back(ogreVec3FromLLetPoint(point));
  }
  return line;
}

Ogre::Vector3 Lanelet2MapHelpers::ogreVec3FromLLetPoint(lanelet::ConstPoint3d point) const {
  using boost::numeric_cast;
  return Ogre::Vector3(numeric_cast<Ogre::Real>(point.x()), numeric_cast<Ogre::Real>(point.y()),
                       numeric_cast<Ogre::Real>(point.z()) * numeric_cast<Ogre::Real>(owner_.rend_opts_.threeD));
}

Ogre::Vector3 Lanelet2MapHelpers::ogreVec3FromLLetTrafficLight(lanelet::ConstPoint3d point, double z_offset) const {
  using boost::numeric_cast;
  return Ogre::Vector3(numeric_cast<Ogre::Real>(point.x()), numeric_cast<Ogre::Real>(point.y()),
                       numeric_cast<Ogre::Real>(point.z() + z_offset));
}

std::vector<Ogre::Vector3> Lanelet2MapHelpers::bufferSegment(const std::vector<Ogre::Vector3>& line, double buffer_length) {
  std::vector<Ogre::Vector3> buffered;
  buffered.reserve(line.size());
  assert(line.size() >= 2);
  for (auto it = line.begin(); it != line.end(); ++it) {
    auto normal = getNormal(it, line.begin(), line.end());
    buffered.push_back(*it + normal * boost::numeric_cast<Ogre::Real>(buffer_length / 2.0));
  }
  for (auto it = line.rbegin(); it != line.rend(); ++it) {
    auto normal = getNormal(it, line.rbegin(), line.rend());
    buffered.push_back(*it + normal * boost::numeric_cast<Ogre::Real>(buffer_length / 2.0));
  }
  return buffered;
}

void Lanelet2MapHelpers::drawLine(
    const std::vector<Ogre::Vector3>& line, Ogre::ManualObject* obj, Ogre::ColourValue color, double width, double z_offset) {
  if (width <= 0 || line.size() < 2) {
    return;
  }
  auto buffered = bufferSegment(line, width);
  if (z_offset != 0.0) {
    for (auto& v : buffered) {
      v.z += boost::numeric_cast<Ogre::Real>(z_offset);
    }
  }
  drawMonoPolygon(buffered, obj, color);
}

void Lanelet2MapHelpers::drawArea(const std::vector<Ogre::Vector3>& line,
                                  Ogre::ManualObject* obj,
                                  Ogre::ColourValue color,
                                  double z_offset) {
  if (line.size() < 2) {
    return;
  }
  if (z_offset != 0.0) {
    std::vector<Ogre::Vector3> elevated;
    elevated.reserve(line.size());
    for (auto v : line) {
      v.z += boost::numeric_cast<Ogre::Real>(z_offset);
      elevated.push_back(v);
    }
    drawMonoPolygon(elevated, obj, color);
    return;
  }
  drawMonoPolygon(line, obj, color);
}

void Lanelet2MapHelpers::drawMonoPolygon(const std::vector<Ogre::Vector3>& poly,
                                         Ogre::ManualObject* obj,
                                         Ogre::ColourValue color) {
  if (poly.size() < 3) {
    return;
  }

  auto it_left = poly.begin();
  auto it_right = --poly.end();
  const auto start_index = obj->getCurrentVertexCount();
  auto count = 0u;
  for (; it_right >= it_left; ++it_left, --it_right) {
    obj->position(*it_left);
    obj->normal(0, 0, 1);
    obj->colour(color);
    if (count >= 2) {
      assert(obj->getCurrentVertexCount() > start_index + count);
      obj->triangle(start_index + count - 2, start_index + count - 1, start_index + count);
    }
    count++;
    obj->position(*it_right);
    obj->normal(0, 0, 1);
    obj->colour(color);
    if (count >= 2) {
      assert(obj->getCurrentVertexCount() > start_index + count);
      obj->triangle(start_index + count, start_index + count - 1, start_index + count - 2);
    }
    count++;
  }
}

void Lanelet2MapHelpers::drawLaneFillStrip(const std::vector<Ogre::Vector3>& left,
                                           const std::vector<Ogre::Vector3>& right,
                                           Ogre::ManualObject* obj,
                                           Ogre::ColourValue color,
                                           double z_offset) {
  if (left.size() < 2 || right.size() < 2) {
    return;
  }

  const size_t n_left = left.size();
  const size_t n_right = right.size();
  const size_t max_steps = std::max(n_left, n_right) - 1;
  for (size_t i = 0; i < max_steps; ++i) {
    double t0 = static_cast<double>(i) / static_cast<double>(max_steps);
    double t1 = static_cast<double>(i + 1) / static_cast<double>(max_steps);

    size_t i_left0 = static_cast<size_t>(t0 * static_cast<double>(n_left - 1));
    size_t i_left1 = std::min(static_cast<size_t>(t1 * static_cast<double>(n_left - 1)), n_left - 1);
    size_t i_right0 = static_cast<size_t>(t0 * static_cast<double>(n_right - 1));
    size_t i_right1 = std::min(static_cast<size_t>(t1 * static_cast<double>(n_right - 1)), n_right - 1);

    Ogre::Vector3 left0 = left[i_left0];
    Ogre::Vector3 left1 = left[i_left1];
    Ogre::Vector3 right0 = right[i_right0];
    Ogre::Vector3 right1 = right[i_right1];
    const auto z_offset_real = boost::numeric_cast<Ogre::Real>(z_offset);
    left0.z += z_offset_real;
    left1.z += z_offset_real;
    right0.z += z_offset_real;
    right1.z += z_offset_real;

    const auto base = obj->getCurrentVertexCount();
    obj->position(left0);
    obj->normal(0, 0, 1);
    obj->colour(color);
    obj->position(right0);
    obj->normal(0, 0, 1);
    obj->colour(color);
    obj->position(left1);
    obj->normal(0, 0, 1);
    obj->colour(color);
    obj->triangle(base + 0, base + 1, base + 2);

    obj->position(left1);
    obj->normal(0, 0, 1);
    obj->colour(color);
    obj->position(right0);
    obj->normal(0, 0, 1);
    obj->colour(color);
    obj->position(right1);
    obj->normal(0, 0, 1);
    obj->colour(color);
    obj->triangle(base + 3, base + 4, base + 5);
  }
}

}  // namespace rviz_rendering::detail
