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

#ifndef LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HELPERS_HPP_
#define LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HELPERS_HPP_

#include "lanelet2_rviz_display/lanelet2_map.hpp"

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>

#include <rviz_rendering/objects/movable_text.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <boost/numeric/conversion/cast.hpp>

#include <algorithm>
#include <cassert>
#include <iterator>
#include <vector>

namespace rviz_rendering::detail {

class Lanelet2MapHelpers {
 public:
  /// Create helper routines bound to the owning map renderer.
  explicit Lanelet2MapHelpers(Lanelet2Map& owner) : owner_(owner) {}

  /// Remove all helper-owned OGRE objects.
  void clearObjects();
  /// Add lanelet boundary lines to a manual object.
  void addLaneletToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);
  /// Add lanelet separator lines to a manual object.
  void addSeperatorToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);
  /// Add area outlines or fills to a manual object.
  void addAreaToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);
  /// Add parking area outlines or fills to a manual object.
  void addParkingAreaToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);
  /// Add a filled lanelet surface to a manual object.
  void addLaneFillToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);
  /// Add a sidewalk surface to a manual object.
  void addSidewalkToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);
  /// Add a crosswalk surface to a manual object.
  void addCrosswalkToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);
  /// Attach regulatory element visuals for a lanelet.
  void addRegulatoryElements(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parent_node);
  /// Attach stop-line reference geometry to a scene node.
  void attachRefLinesToSceneNode(std::vector<lanelet::ConstLineString3d>& stop_lines, Ogre::SceneNode* parent_node);
  /// Attach traffic light geometry to a scene node.
  void attachTrafficLightsToSceneNode(std::vector<lanelet::ConstPolygon3d>& traffic_lights, Ogre::SceneNode* parent_node);
  /// Attach the lanelet ID text to a scene node.
  void attachLaneletIdToSceneNode(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parent_node);

 private:
  /// Convert a Lanelet2 line string to OGRE points.
  std::vector<Ogre::Vector3> ogreLineFromLLetLineString(const lanelet::ConstLineString3d& line_string) const;
  /// Convert a Lanelet2 polygon to OGRE points.
  std::vector<Ogre::Vector3> ogreLineFromLLetPolygon(const lanelet::CompoundPolygon3d& polygon) const;
  /// Convert a traffic light polygon to OGRE points.
  std::vector<Ogre::Vector3> ogreLineFromLLetTrafficLight(const lanelet::ConstPolygon3d& polygon) const;
  /// Convert Lanelet2 points to OGRE points.
  std::vector<Ogre::Vector3> ogreLineFromLLetPts(const lanelet::ConstPoints3d& pts_vector) const;
  /// Convert a Lanelet2 point to an OGRE point.
  Ogre::Vector3 ogreVec3FromLLetPoint(lanelet::ConstPoint3d point) const;
  /// Convert a traffic light point to an OGRE point with a height offset.
  Ogre::Vector3 ogreVec3FromLLetTrafficLight(lanelet::ConstPoint3d point, double z_offset) const;

  /// Create a buffered segment around a line.
  std::vector<Ogre::Vector3> bufferSegment(const std::vector<Ogre::Vector3>& line, double buffer_length);

  /// Return the local 2D normal at an iterator position.
  template <typename Iter>
  static Ogre::Vector3 getNormal(Iter it, Iter begin, Iter end) {
    Ogre::Vector3 zero(0, 0, 0);
    auto dir_before = zero;
    int i = 1;
    while (it != begin && dir_before == zero) {
      dir_before = *it - *std::prev(it, i);
      i++;
    }

    auto dir_after = zero;
    i = 1;
    while (std::next(it) != end && dir_after == zero) {
      dir_after = *std::next(it, i) - *it;
      i++;
    }

    if (dir_before == zero && dir_after == zero) {
      return zero;
    }
    if (dir_after == zero) {
      dir_after = dir_before;
    }
    if (dir_before == zero) {
      dir_before = dir_after;
    }

    auto dir_combined = dir_after * (1 / dir_after.length()) + dir_before * (1 / dir_before.length());
    dir_combined *= 1 / dir_combined.length();
    return Ogre::Vector3(dir_combined.y, -dir_combined.x, 0);
  }

  /// Draw a line strip into a manual object.
  void drawLine(const std::vector<Ogre::Vector3>& line,
                Ogre::ManualObject* obj,
                Ogre::ColourValue color = Ogre::ColourValue::White,
                double width = 0.1,
                double z_offset = 0.0);
  /// Draw an area outline or fill into a manual object.
  void drawArea(const std::vector<Ogre::Vector3>& line,
                Ogre::ManualObject* obj,
                Ogre::ColourValue color = Ogre::ColourValue::White,
                double z_offset = 0.0);
  /// Draw a single polygon into a manual object.
  void drawMonoPolygon(const std::vector<Ogre::Vector3>& poly,
                       Ogre::ManualObject* obj,
                       Ogre::ColourValue color = Ogre::ColourValue::White);
  /// Draw a filled strip between lanelet boundaries.
  void drawLaneFillStrip(const std::vector<Ogre::Vector3>& left,
                         const std::vector<Ogre::Vector3>& right,
                         Ogre::ManualObject* obj,
                         Ogre::ColourValue color,
                         double z_offset = 0.0);

  Lanelet2Map& owner_;
};

}  // namespace rviz_rendering::detail

#endif  // LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HELPERS_HPP_
