/*
 *
 * This implementation is partly based on the ROS1 lanelet_rviz_plugin from FZI & KIT and the RViz Rendering "Grid" and the RViz Default Plugins "Grid Display"
 * https://github.com/coincar-sim/lanelet_rviz_plugin_ros/
 * https://github.com/ros2/rviz/tree/rolling/rviz_default_plugins
 * https://github.com/ros2/rviz/blob/rolling/rviz_rendering
 *
 * NOTE THE COPYRIGHT AND LICENSE BEFORE ANY DISTRIBUTION
 *
 * AUTHOR: Guido Küppers (guido.kueppers@ika.rwth-aachen.de)
 *
 */

#ifndef LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HPP_
#define LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HPP_

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#include <rviz_rendering/objects/movable_text.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <boost/numeric/conversion/cast.hpp>

namespace rviz_rendering {

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

using ClassifiedMovableObject = std::pair<ObjectClassification, Ogre::MovableObject *>;
/**
     * \class Lanelet2Map
     * \brief Displays a Lanelet2Map
     *
     * Displays a Lanelet2Map.
     */
class Lanelet2Map {
 public:
  static int manual_object_counter_;
  struct RenderingOptions {
    // General
    bool threeD = true;

    // Line-Strings
    bool renderLaneletLinestrings = true;
    double linestringWidth = 0.1;
    Ogre::ColourValue colorLeft{Ogre::ColourValue(1.0, 1.0, 1.0, 1.0)};   // white
    Ogre::ColourValue colorRight{Ogre::ColourValue(1.0, 1.0, 1.0, 1.0)};  // white

    // Separator
    bool renderLaneletSeparators = false;
    Ogre::ColourValue colorSeperator{Ogre::ColourValue(0.1, 0.1, 0.9, 1.0)};  // blue
    double seperatorWidth = 0.2;

    // StopLines
    bool renderStopLines = true;
    Ogre::ColourValue colorStopLine{Ogre::ColourValue(1.0, 0.1, 0.1, 1.0)};  // red
    double stopLineWidth = 0.2;

    // Traffic lights
    bool renderTrafficLights = true;
    Ogre::ColourValue colorTrafficLight{Ogre::ColourValue(0.4, 0.4, 0.4, 1.0)};  // gray
    double trafficLightHeightAboveGround = 3.0;

    // Lane fills
    bool renderLaneletFills = true;
    Ogre::ColourValue colorLaneFill{Ogre::ColourValue(0.22, 0.22, 0.22, 0.95)};  // dark gray

    // Areas
    bool renderAreas = true;
    Ogre::ColourValue colorArea{Ogre::ColourValue(0.9, 0.5, 0.1, 1.0)};  // orange
    double areaWidth = 0.3;
    bool fillArea = false;

    // Parking
    bool renderParking = true;
    Ogre::ColourValue colorParking{Ogre::ColourValue(0.0, 0.7, 0.3, 1.0)};  // green
    double parkingWidth = 0.3;
    bool fillParking = true;

    // Pedestrian features
    bool renderSidewalks = true;
    Ogre::ColourValue colorSidewalk{Ogre::ColourValue(0.75, 0.75, 0.75, 0.7)};  // light gray
    bool renderCrosswalks = true;
    Ogre::ColourValue colorCrosswalk{Ogre::ColourValue(1.0, 1.0, 1.0, 0.85)};  // white

    // IDs
    bool renderLaneletIds = false;
    Ogre::ColourValue colorLaneletId{Ogre::ColourValue(1.0, 1.0, 1.0, 1.0)};  // white
    double characterHeight = 1.0;

    // Z offsets (meters) to avoid z-fighting, increasing = rendered above
    // Bumped to stronger defaults for stability across camera ranges
    double zLaneFill = 0.02;
    double zSidewalk = 0.03;
    double zAreas = 0.032;
    double zParking = 0.034;
    double zCrosswalk = 0.05;
    double zRoadLines = 0.06;
    double zStopLine = 0.08;
    double zSeparator = 0.085;
  };

  Lanelet2Map(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node, Lanelet2Map::RenderingOptions rend_opts,
              lanelet::LaneletMapConstPtr map_ptr);
  ~Lanelet2Map();

  void clearObjects();

  /**
   * \brief Get the Ogre scene node associated with this ll2-map
   *
   * @return The Ogre scene node associated with this ll2-map
   */
  Ogre::SceneNode *getSceneNode() { return scene_node_; }

  void updateMap(RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr);
  void updateVisibility(const RenderingOptions &rend_opts);

 private:
  Lanelet2Map::RenderingOptions rend_opts_;  // Rendering options for this ll2-map

  Ogre::SceneManager *scene_manager_;
  Ogre::SceneNode *scene_node_;  // The scene node that this ll2-map is attached to
  std::vector<ClassifiedMovableObject> objects_;

  std::shared_ptr<Ogre::Material> material_surface_;
  std::shared_ptr<Ogre::Material> material_line_;

  void create(lanelet::LaneletMapConstPtr map_ptr);

  void addLaneletToManualObject(const lanelet::ConstLanelet &lanelet, Ogre::ManualObject *manual);
  void addSeperatorToManualObject(const lanelet::ConstLanelet &lanelet, Ogre::ManualObject *manual);
  void addAreaToManualObject(const lanelet::ConstArea &area, Ogre::ManualObject *manual);
  void addParkingAreaToManualObject(const lanelet::ConstArea &area, Ogre::ManualObject *manual);
  void addLaneFillToManualObject(const lanelet::ConstLanelet &lanelet, Ogre::ManualObject *manual);
  void addSidewalkToManualObject(const lanelet::ConstArea &area, Ogre::ManualObject *manual);
  void addCrosswalkToManualObject(const lanelet::ConstArea &area, Ogre::ManualObject *manual);

  void addRegulatoryElements(const lanelet::ConstLanelet &lanelet, Ogre::SceneNode *parentNode);

  void attachRefLinesToSceneNode(std::vector<lanelet::ConstLineString3d> &stopLines, Ogre::SceneNode *parentNode);
  void attachTrafficLightsToSceneNode(std::vector<lanelet::ConstPolygon3d> &trafficLights, Ogre::SceneNode *parentNode);
  void attachLaneletIdToSceneNode(const lanelet::ConstLanelet &lanelet, Ogre::SceneNode *parentNode);

  std::vector<Ogre::Vector3> ogreLineFromLLetLineString(const lanelet::ConstLineString3d &lineString) const;
  std::vector<Ogre::Vector3> ogreLineFromLLetPolygon(const lanelet::CompoundPolygon3d &polygon) const;
  std::vector<Ogre::Vector3> ogreLineFromLLetTrafficLight(const lanelet::ConstPolygon3d &polygon3d) const;
  std::vector<Ogre::Vector3> ogreLineFromLLetPts(const lanelet::ConstPoints3d &ptsVector) const;
  Ogre::Vector3 ogreVec3FromLLetPoint(const lanelet::ConstPoint3d point) const;
  Ogre::Vector3 ogreVec3FromLLetTrafficLight(const lanelet::ConstPoint3d point, const double zOffset) const;

  void updateVisibility(ObjectClassification classification, bool visible);

  // Helper functions
  std::vector<Ogre::Vector3> bufferSegment(const std::vector<Ogre::Vector3> &line, double buffer_length);
  template <typename Iter>
  Ogre::Vector3 getNormal(Iter it, Iter begin, Iter end);
  void drawLine(const std::vector<Ogre::Vector3> &line, Ogre::ManualObject *obj,
                Ogre::ColourValue color = Ogre::ColourValue::White, double width = 0.1, double zOffset = 0.0);
  void drawArea(const std::vector<Ogre::Vector3> &line, Ogre::ManualObject *obj,
                Ogre::ColourValue color = Ogre::ColourValue::White, double zOffset = 0.0);
  void drawMonoPolygon(const std::vector<Ogre::Vector3> &poly, Ogre::ManualObject *obj,
                       Ogre::ColourValue color = Ogre::ColourValue::White);

  void drawLaneFillStrip(const std::vector<Ogre::Vector3> &left,
                         const std::vector<Ogre::Vector3> &right,
                         Ogre::ManualObject *obj,
                         Ogre::ColourValue color,
                         double zOffset = 0.0);
};

}  // namespace rviz_rendering

#endif  // LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HPP_
