// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

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

using ClassifiedMovableObject = std::pair<ObjectClassification, Ogre::MovableObject*>;
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

  /**
   * @brief Creates a renderable Lanelet2 map attached to an Ogre scene node.
   *
   * @param manager Ogre scene manager used to create render objects
   * @param parent_node parent scene node that owns the map scene node
   * @param rend_opts initial rendering options
   * @param map_ptr map to render initially
   */
  Lanelet2Map(Ogre::SceneManager* manager,
              Ogre::SceneNode* parent_node,
              Lanelet2Map::RenderingOptions rend_opts,
              lanelet::LaneletMapConstPtr map_ptr);

  /**
   * @brief Destroys the rendered map and all Ogre objects owned by it.
   */
  ~Lanelet2Map();

  /**
   * @brief Removes all currently attached Ogre objects from the scene node.
   */
  void clearObjects();

  /**
   * \brief Get the Ogre scene node associated with this ll2-map
   *
   * @return The Ogre scene node associated with this ll2-map
   */
  Ogre::SceneNode* getSceneNode() { return scene_node_; }

  /**
   * @brief Replaces the displayed map and rendering configuration.
   *
   * @param rend_opts new rendering options
   * @param map_ptr map to render
   */
  void updateMap(RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr);

  /**
   * @brief Updates visibility of already-created objects without rebuilding geometry.
   *
   * @param rend_opts rendering options that define which classes should be visible
   */
  void updateVisibility(const RenderingOptions& rend_opts);

 private:
  Lanelet2Map::RenderingOptions rend_opts_;  // Rendering options for this ll2-map

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_node_;  // The scene node that this ll2-map is attached to
  std::vector<ClassifiedMovableObject> objects_;

  std::shared_ptr<Ogre::Material> material_surface_;
  std::shared_ptr<Ogre::Material> material_line_;

  /**
   * @brief Builds the complete Ogre representation of a Lanelet2 map.
   *
   * @param map_ptr map to render
   */
  void create(lanelet::LaneletMapConstPtr map_ptr);

  /**
   * @brief Adds the lanelet boundary geometry for a single lanelet.
   *
   * @param lanelet lanelet to render
   * @param manual manual object receiving the geometry
   */
  void addLaneletToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);

  /**
   * @brief Adds separator markings associated with a lanelet.
   *
   * @param lanelet lanelet whose separators are rendered
   * @param manual manual object receiving the geometry
   */
  void addSeperatorToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);

  /**
   * @brief Adds an area outline or fill geometry.
   *
   * @param area area primitive to render
   * @param manual manual object receiving the geometry
   */
  void addAreaToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);

  /**
   * @brief Adds a parking area outline or fill geometry.
   *
   * @param area parking area primitive to render
   * @param manual manual object receiving the geometry
   */
  void addParkingAreaToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);

  /**
   * @brief Adds a filled road-surface polygon for a lanelet.
   *
   * @param lanelet lanelet to render
   * @param manual manual object receiving the geometry
   */
  void addLaneFillToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);

  /**
   * @brief Adds a filled sidewalk polygon.
   *
   * @param area sidewalk area to render
   * @param manual manual object receiving the geometry
   */
  void addSidewalkToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);

  /**
   * @brief Adds a filled crosswalk polygon.
   *
   * @param area crosswalk area to render
   * @param manual manual object receiving the geometry
   */
  void addCrosswalkToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);

  /**
   * @brief Adds rendered regulatory elements belonging to a lanelet.
   *
   * @param lanelet lanelet whose regulatory elements are processed
   * @param parentNode scene node that receives the created objects
   */
  void addRegulatoryElements(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parentNode);

  /**
   * @brief Attaches stop-line reference lines to the scene.
   *
   * @param stopLines stop-line geometries to render
   * @param parentNode scene node that receives the created objects
   */
  void attachRefLinesToSceneNode(std::vector<lanelet::ConstLineString3d>& stopLines, Ogre::SceneNode* parentNode);

  /**
   * @brief Attaches traffic-light geometries to the scene.
   *
   * @param trafficLights traffic-light polygons to render
   * @param parentNode scene node that receives the created objects
   */
  void attachTrafficLightsToSceneNode(std::vector<lanelet::ConstPolygon3d>& trafficLights, Ogre::SceneNode* parentNode);

  /**
   * @brief Attaches a text label showing a lanelet identifier.
   *
   * @param lanelet lanelet whose identifier is shown
   * @param parentNode scene node that receives the created objects
   */
  void attachLaneletIdToSceneNode(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parentNode);

  /**
   * @brief Converts a lanelet line string to Ogre coordinates.
   *
   * @param lineString line string to convert
   * @return converted polyline in Ogre coordinates
   */
  std::vector<Ogre::Vector3> ogreLineFromLLetLineString(const lanelet::ConstLineString3d& lineString) const;

  /**
   * @brief Converts a lanelet polygon boundary to Ogre coordinates.
   *
   * @param polygon polygon to convert
   * @return converted polygon outline in Ogre coordinates
   */
  std::vector<Ogre::Vector3> ogreLineFromLLetPolygon(const lanelet::CompoundPolygon3d& polygon) const;

  /**
   * @brief Converts a traffic-light polygon to a representative Ogre polyline.
   *
   * @param polygon3d polygon to convert
   * @return converted polyline in Ogre coordinates
   */
  std::vector<Ogre::Vector3> ogreLineFromLLetTrafficLight(const lanelet::ConstPolygon3d& polygon3d) const;

  /**
   * @brief Converts a point sequence to Ogre coordinates.
   *
   * @param ptsVector points to convert
   * @return converted points in Ogre coordinates
   */
  std::vector<Ogre::Vector3> ogreLineFromLLetPts(const lanelet::ConstPoints3d& ptsVector) const;

  /**
   * @brief Converts a Lanelet2 point to an Ogre vector.
   *
   * @param point point to convert
   * @return corresponding Ogre vector
   */
  Ogre::Vector3 ogreVec3FromLLetPoint(const lanelet::ConstPoint3d point) const;

  /**
   * @brief Converts a traffic-light point to an Ogre vector with an added z offset.
   *
   * @param point point to convert
   * @param zOffset offset added to the z coordinate
   * @return corresponding Ogre vector
   */
  Ogre::Vector3 ogreVec3FromLLetTrafficLight(const lanelet::ConstPoint3d point, const double zOffset) const;

  /**
   * @brief Updates visibility for one rendered object class.
   *
   * @param classification object category to update
   * @param visible whether objects of that class should be shown
   */
  void updateVisibility(ObjectClassification classification, bool visible);

  // Helper functions
  /**
   * @brief Creates a buffered strip around a polyline.
   *
   * @param line input polyline
   * @param buffer_length half-width of the buffer
   * @return buffered polygon points
   */
  std::vector<Ogre::Vector3> bufferSegment(const std::vector<Ogre::Vector3>& line, double buffer_length);

  /**
   * @brief Computes an approximate normal vector for a polyline vertex.
   *
   * @tparam Iter iterator type over `Ogre::Vector3`
   * @param it iterator to the current point
   * @param begin begin iterator of the line
   * @param end end iterator of the line
   * @return normalized lateral vector at the current point
   */
  template <typename Iter>
  Ogre::Vector3 getNormal(Iter it, Iter begin, Iter end);

  /**
   * @brief Draws a thick polyline into an Ogre manual object.
   *
   * @param line line to draw
   * @param obj manual object receiving the geometry
   * @param color line color
   * @param width rendered line width
   * @param zOffset vertical offset applied to all vertices
   */
  void drawLine(const std::vector<Ogre::Vector3>& line,
                Ogre::ManualObject* obj,
                Ogre::ColourValue color = Ogre::ColourValue::White,
                double width = 0.1,
                double zOffset = 0.0);

  /**
   * @brief Draws a filled polygon area into an Ogre manual object.
   *
   * @param line polygon boundary
   * @param obj manual object receiving the geometry
   * @param color fill color
   * @param zOffset vertical offset applied to all vertices
   */
  void drawArea(const std::vector<Ogre::Vector3>& line,
                Ogre::ManualObject* obj,
                Ogre::ColourValue color = Ogre::ColourValue::White,
                double zOffset = 0.0);

  /**
   * @brief Draws a simple polygon without holes.
   *
   * @param poly polygon vertices
   * @param obj manual object receiving the geometry
   * @param color fill color
   */
  void drawMonoPolygon(const std::vector<Ogre::Vector3>& poly,
                       Ogre::ManualObject* obj,
                       Ogre::ColourValue color = Ogre::ColourValue::White);

  /**
   * @brief Draws a triangle strip between left and right lane boundaries.
   *
   * @param left left boundary points
   * @param right right boundary points
   * @param obj manual object receiving the geometry
   * @param color fill color
   * @param zOffset vertical offset applied to all vertices
   */
  void drawLaneFillStrip(const std::vector<Ogre::Vector3>& left,
                         const std::vector<Ogre::Vector3>& right,
                         Ogre::ManualObject* obj,
                         Ogre::ColourValue color,
                         double zOffset = 0.0);
};

}  // namespace rviz_rendering

#endif  // LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HPP_
