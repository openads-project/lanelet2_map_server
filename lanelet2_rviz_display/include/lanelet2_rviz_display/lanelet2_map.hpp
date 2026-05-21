// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#ifndef LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HPP_
#define LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HPP_

#include <OgreColourValue.h>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <utility>
#include <vector>

namespace Ogre {
class Material;
class MovableObject;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace rviz_rendering {

namespace detail {
class Lanelet2MapHelpers;
}  // namespace detail

enum class ObjectClassification {
  UNKOWN,
  MAP,
  LANELETID,
  AREA,
  PARKINGAREA,
  SEPERATOR,
  STOPLINE,
  TRAFFICLIGHT,
  LANEFILL,
  SIDEWALK,
  CROSSWALK
};

using ClassifiedMovableObject = std::pair<ObjectClassification, Ogre::MovableObject*>;

class Lanelet2Map {
  friend class detail::Lanelet2MapHelpers;

 public:
  static int manual_object_counter_;

  struct RenderingOptions {
    bool threeD = true;

    bool renderLaneletLinestrings = true;
    double linestringWidth = 0.1;
    Ogre::ColourValue colorLeft{Ogre::ColourValue(1.0, 1.0, 1.0, 1.0)};
    Ogre::ColourValue colorRight{Ogre::ColourValue(1.0, 1.0, 1.0, 1.0)};

    bool renderLaneletSeparators = false;
    Ogre::ColourValue colorSeperator{Ogre::ColourValue(0.1, 0.1, 0.9, 1.0)};
    double seperatorWidth = 0.2;

    bool renderStopLines = true;
    Ogre::ColourValue colorStopLine{Ogre::ColourValue(1.0, 0.1, 0.1, 1.0)};
    double stopLineWidth = 0.2;

    bool renderTrafficLights = true;
    Ogre::ColourValue colorTrafficLight{Ogre::ColourValue(0.4, 0.4, 0.4, 1.0)};
    double trafficLightHeightAboveGround = 3.0;

    bool renderLaneletFills = true;
    Ogre::ColourValue colorLaneFill{Ogre::ColourValue(0.22, 0.22, 0.22, 0.95)};

    bool renderAreas = true;
    Ogre::ColourValue colorArea{Ogre::ColourValue(0.9, 0.5, 0.1, 1.0)};
    double areaWidth = 0.3;
    bool fillArea = false;

    bool renderParking = true;
    Ogre::ColourValue colorParking{Ogre::ColourValue(0.0, 0.7, 0.3, 1.0)};
    double parkingWidth = 0.3;
    bool fillParking = true;

    bool renderSidewalks = true;
    Ogre::ColourValue colorSidewalk{Ogre::ColourValue(0.75, 0.75, 0.75, 0.7)};
    bool renderCrosswalks = true;
    Ogre::ColourValue colorCrosswalk{Ogre::ColourValue(1.0, 1.0, 1.0, 0.85)};

    bool renderLaneletIds = false;
    Ogre::ColourValue colorLaneletId{Ogre::ColourValue(1.0, 1.0, 1.0, 1.0)};
    double characterHeight = 1.0;

    double zLaneFill = 0.02;
    double zSidewalk = 0.03;
    double zAreas = 0.032;
    double zParking = 0.034;
    double zCrosswalk = 0.05;
    double zRoadLines = 0.06;
    double zStopLine = 0.08;
    double zSeparator = 0.085;
  };

  /// Create an RViz scene representation for a lanelet map.
  Lanelet2Map(Ogre::SceneManager* manager,
              Ogre::SceneNode* parent_node,
              Lanelet2Map::RenderingOptions rend_opts,
              lanelet::LaneletMapConstPtr map_ptr);

  /// Destroy the scene representation and its OGRE objects.
  ~Lanelet2Map();

  /// Remove all OGRE objects owned by this map.
  void clearObjects();

  /// Return the root scene node used by this map.
  Ogre::SceneNode* getSceneNode() { return scene_node_; }

  /// Replace the rendered map and apply the given rendering options.
  void updateMap(RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr);

  /// Update object visibility from the supplied rendering options.
  void updateVisibility(const RenderingOptions& rend_opts);

 private:
  /// Create OGRE objects for all supported lanelet map layers.
  void create(lanelet::LaneletMapConstPtr map_ptr);
  /// Set visibility for one object classification.
  void updateVisibility(ObjectClassification classification, bool visible);

  Lanelet2Map::RenderingOptions rend_opts_;

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_node_;
  std::vector<ClassifiedMovableObject> objects_;

  std::shared_ptr<Ogre::Material> material_surface_;
  std::shared_ptr<Ogre::Material> material_line_;
  std::unique_ptr<detail::Lanelet2MapHelpers> bsd_helpers_;
};

}  // namespace rviz_rendering

#endif  // LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HPP_
