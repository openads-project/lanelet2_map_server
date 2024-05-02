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

#include "lanelet2_rviz_display/lanelet2_map.hpp"

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreVector.h>

namespace rviz_rendering {

int Lanelet2Map::manual_object_counter_{0};

Lanelet2Map::Lanelet2Map(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node,
                         Lanelet2Map::RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr)
    : rend_opts_(rend_opts), scene_manager_(manager) {
  std::string ll_map_name = "Lanelet2Map";

  if (!parent_node) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  std::string map_material_name = ll_map_name + "Material";

  auto retrieve_result = Ogre::MaterialManager::getSingleton().createOrRetrieve(map_material_name, "rviz_rendering");
  auto material = std::dynamic_pointer_cast<Ogre::Material>(retrieve_result.first);
  if (material) {
    material_ = material;
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(false);
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->getTechnique(0)->setDepthWriteEnabled(true);
    material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT + Ogre::TVC_DIFFUSE);
    material_->getTechnique(0)->setCullingMode(Ogre::CULL_NONE);  // No culling
    create(map_ptr);
    updateVisibility(rend_opts_);
  }
}

Lanelet2Map::~Lanelet2Map() {
  clearObjects();

  // destroy all child scene nodes
  scene_node_->removeAndDestroyAllChildren();
  // self detach from parent_scene_node
  scene_node_->getParentSceneNode()->removeChild(scene_node_);
  // delete sceneNode_
  scene_manager_->destroySceneNode(scene_node_);

  material_->unload();
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L66
void Lanelet2Map::clearObjects() {
  for (auto object : objects_) {
    switch (object.first) {
      case ObjectClassification::MAP: {
        auto man_object = dynamic_cast<Ogre::ManualObject *>(object.second);
        if (man_object) {
          man_object->detachFromParent();
          scene_manager_->destroyManualObject(man_object);
        }
        break;
      }

      case ObjectClassification::LANELETID: {
        auto mov_text = dynamic_cast<rviz_rendering::MovableText *>(object.second);
        if (mov_text) {
          mov_text->detachFromParent();
          delete mov_text;
        }
        break;
      }

      case ObjectClassification::AREA: {
        auto man_object = dynamic_cast<Ogre::ManualObject *>(object.second);
        if (man_object) {
          man_object->detachFromParent();
          scene_manager_->destroyManualObject(man_object);
        }
        break;
      }

      case ObjectClassification::PARKINGAREA: {
        auto man_object = dynamic_cast<Ogre::ManualObject *>(object.second);
        if (man_object) {
          man_object->detachFromParent();
          scene_manager_->destroyManualObject(man_object);
        }
        break;
      }

      case ObjectClassification::SEPERATOR: {
        auto man_object = dynamic_cast<Ogre::ManualObject *>(object.second);
        if (man_object) {
          man_object->detachFromParent();
          scene_manager_->destroyManualObject(man_object);
        }
        break;
      }

      case ObjectClassification::STOPLINE: {
        auto man_object = dynamic_cast<Ogre::ManualObject *>(object.second);
        if (man_object) {
          man_object->detachFromParent();
          scene_manager_->destroyManualObject(man_object);
        }
        break;
      }

      case ObjectClassification::TRAFFICLIGHT: {
        auto man_object = dynamic_cast<Ogre::ManualObject *>(object.second);
        if (man_object) {
          man_object->detachFromParent();
          scene_manager_->destroyManualObject(man_object);
        }
        break;
      }

      default: {
        if (object.second) {
          object.second->detachFromParent();
          delete object.second;
        }
      }
    }
  }
  manual_object_counter_ = 0;
  objects_.clear();
}

void Lanelet2Map::updateMap(Lanelet2Map::RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr) {
  rend_opts_ = rend_opts;
  clearObjects();                // Clear all old objects
  create(map_ptr);               // Create new objects from map
  updateVisibility(rend_opts_);  // Hide all objects that are not supposed to be visible
}

void Lanelet2Map::updateVisibility(const RenderingOptions &rend_opts) {
  rend_opts_ = rend_opts;
  updateVisibility(ObjectClassification::MAP, rend_opts_.renderLaneletLinestrings);
  updateVisibility(ObjectClassification::LANELETID, rend_opts_.renderLaneletIds);
  updateVisibility(ObjectClassification::AREA, rend_opts_.renderAreas);
  updateVisibility(ObjectClassification::PARKINGAREA, rend_opts_.renderParking);
  updateVisibility(ObjectClassification::SEPERATOR, rend_opts_.renderLaneletSeparators);
  updateVisibility(ObjectClassification::STOPLINE, rend_opts_.renderStopLines);
  updateVisibility(ObjectClassification::TRAFFICLIGHT, rend_opts_.renderTrafficLights);
}

void Lanelet2Map::updateVisibility(ObjectClassification classification, bool visible) {
  for (auto object : objects_) {
    if (object.first == classification) {
      object.second->setVisible(visible);
    }
  }
}

void Lanelet2Map::create(lanelet::LaneletMapConstPtr map_ptr) {
  // Create manual objects that will be attached to the scene-node
  Ogre::ManualObject *mapManualObject =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject *seperatorManualObject =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject *areaManualObject =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject *parkingManualObject =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));

  if (map_ptr != nullptr) {
    // Attach Lanelet to manual object
    mapManualObject->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
    seperatorManualObject->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
    areaManualObject->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
    parkingManualObject->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
    // Iterate through all lanelets in map-graph
    for (const lanelet::ConstLanelet &lanelet : map_ptr->laneletLayer) {
      addLaneletToManualObject(lanelet, mapManualObject);
      addSeperatorToManualObject(lanelet, seperatorManualObject);
      addRegulatoryElements(lanelet, scene_node_);
      attachLaneletIdToSceneNode(lanelet, scene_node_);
    }
    for (const lanelet::ConstArea &area : map_ptr->areaLayer) {
      auto attributes = area.attributes();
      if (attributes[lanelet::AttributeName::Subtype] == lanelet::AttributeValueString::Parking) {
        addParkingAreaToManualObject(area, parkingManualObject);
      } else {
        addAreaToManualObject(area, areaManualObject);
      }
    }
    mapManualObject->end();
    seperatorManualObject->end();
    areaManualObject->end();
    parkingManualObject->end();
    // attach manual object to scene node
    if (mapManualObject->getNumSections()) {
      scene_node_->attachObject(mapManualObject);
      // save ptr to manual object (needed later for deletion)
      objects_.push_back(std::make_pair(ObjectClassification::MAP, mapManualObject));
    } else {
      scene_manager_->destroyManualObject(mapManualObject);
    }

    if (seperatorManualObject->getNumSections()) {
      scene_node_->attachObject(seperatorManualObject);
      // save ptr to manual object (needed later for deletion)
      objects_.push_back(std::make_pair(ObjectClassification::SEPERATOR, seperatorManualObject));
    } else {
      scene_manager_->destroyManualObject(seperatorManualObject);
    }

    if (areaManualObject->getNumSections()) {
      scene_node_->attachObject(areaManualObject);
      // save ptr to manual object (needed later for deletion)
      objects_.push_back(std::make_pair(ObjectClassification::AREA, areaManualObject));
    } else {
      scene_manager_->destroyManualObject(areaManualObject);
    }

    if (parkingManualObject->getNumSections()) {
      scene_node_->attachObject(parkingManualObject);
      // save ptr to manual object (needed later for deletion)
      objects_.push_back(std::make_pair(ObjectClassification::PARKINGAREA, parkingManualObject));
    } else {
      scene_manager_->destroyManualObject(parkingManualObject);
    }
  }
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L339
void Lanelet2Map::addLaneletToManualObject(const lanelet::ConstLanelet &lanelet, Ogre::ManualObject *manual) {
  // get line from left Linestrip points
  auto leftbound = lanelet.leftBound();
  auto lineLeft = ogreLineFromLLetLineString(leftbound);
  // draw line as Ogre Object
  drawLine(lineLeft, manual, rend_opts_.colorLeft, rend_opts_.linestringWidth);
  // get line from right Linestrip points
  auto rightbound = lanelet.rightBound();
  auto lineRight = ogreLineFromLLetLineString(rightbound);
  // draw line as Ogre Object
  drawLine(lineRight, manual, rend_opts_.colorRight, rend_opts_.linestringWidth);
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L384
void Lanelet2Map::addSeperatorToManualObject(const lanelet::ConstLanelet &lanelet, Ogre::ManualObject *manual) {
  // get line from first Point of the left Linestrip to the first Point of the
  // right Linestrip
  lanelet::ConstPoints3d pointsF;
  pointsF.push_back(lanelet.leftBound().front());
  pointsF.push_back(lanelet.rightBound().front());
  // draw line as Ogre Object
  auto line = ogreLineFromLLetPts(pointsF);
  drawLine(line, manual, rend_opts_.colorSeperator, rend_opts_.seperatorWidth);

  // get line from last Point of the left Linestrip to the last Point of the
  // right Linestrip
  lanelet::ConstPoints3d pointsL;
  pointsF.push_back(lanelet.leftBound().front());
  pointsF.push_back(lanelet.rightBound().front());
  // draw line as Ogre Object
  line = ogreLineFromLLetPts(pointsL);
  drawLine(line, manual, rend_opts_.colorSeperator, rend_opts_.seperatorWidth);
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L352
void Lanelet2Map::addAreaToManualObject(const lanelet::ConstArea &area, Ogre::ManualObject *manual) {
  // get outer boundary of area
  // auto outerbound = area.outerBound(); //would return a vector, therefore the conversion to polygon to get a
  // lanelet-style ConstLineString3d

  const lanelet::CompoundPolygon3d outerPolygon = area.outerBoundPolygon();
  const auto arealine = ogreLineFromLLetPolygon(outerPolygon);

  // draw polygon as Ogre Object
  if (rend_opts_.fillArea) {
    drawArea(arealine, manual, rend_opts_.colorArea);
  } else {
    drawLine(arealine, manual, rend_opts_.colorArea, rend_opts_.areaWidth);
  }
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L368
void Lanelet2Map::addParkingAreaToManualObject(const lanelet::ConstArea &area, Ogre::ManualObject *manual) {
  // get outer boundary of area
  // auto outerbound = area.outerBound(); //would return a vector, therefore the conversion to polygon to get a
  // lanelet-style ConstLineString3d

  const lanelet::CompoundPolygon3d outerPolygon = area.outerBoundPolygon();
  const auto arealine = ogreLineFromLLetPolygon(outerPolygon);

  // draw polygon as Ogre Object
  if (rend_opts_.fillParking) {
    drawArea(arealine, manual, rend_opts_.colorParking);
  } else {
    drawLine(arealine, manual, rend_opts_.colorParking, rend_opts_.parkingWidth);
  }
}

void Lanelet2Map::addRegulatoryElements(const lanelet::ConstLanelet &lanelet, Ogre::SceneNode *parentNode) {
  // create child SceneNode. Only SceneNodes can be positioned.
  Ogre::SceneNode *regulatoryElementsNode = parentNode->createChildSceneNode();
  // get pointer to regulatory elements
  auto regulatoryElements = lanelet.regulatoryElements();
  auto trafficLightRegelems = lanelet.regulatoryElementsAs<lanelet::TrafficLight>();

  // loop  over the regulatory elements
  if (rend_opts_.renderStopLines) {
    for (auto &&regElement : regulatoryElements) {
      // Get reference lines
      auto refLines = regElement.get()->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
      attachRefLinesToSceneNode(refLines, regulatoryElementsNode);
    }
  }
  // loop over traffic lights (RoleName::Refers applies also to speedlimits which are part of the above loop)
  if (rend_opts_.renderTrafficLights) {
    for (auto &&trafficLight : trafficLightRegelems) {
      // ConstPolygon3d for Roadsigns pretend to be traffic lights, ConstLineString3d for actual traffic lights
      auto trafficLights = trafficLight.get()->getParameters<lanelet::ConstPolygon3d>(lanelet::RoleName::Refers);
      attachTrafficLightsToSceneNode(trafficLights, regulatoryElementsNode);
    }
  }
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L278
void Lanelet2Map::attachRefLinesToSceneNode(std::vector<lanelet::ConstLineString3d> &stopLines,
                                            Ogre::SceneNode *parentNode) {
  // Create Manual Object, RefLines will be created as Manual Object using the
  // ogre_helper::drawLine helper function
  Ogre::ManualObject *stopLinesManualObject =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  stopLinesManualObject->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  for (auto &&stopLine : stopLines) {
    auto line = Lanelet2Map::ogreLineFromLLetLineString(stopLine);
    drawLine(line, stopLinesManualObject, rend_opts_.colorStopLine, rend_opts_.stopLineWidth);
  }
  if (stopLinesManualObject->getNumSections()) {
    parentNode->attachObject(stopLinesManualObject);
    // save ptr to manual object (needed later for deletion)
    objects_.push_back(std::make_pair(ObjectClassification::STOPLINE, stopLinesManualObject));
  }
  stopLinesManualObject->end();
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L298
void Lanelet2Map::attachTrafficLightsToSceneNode(std::vector<lanelet::ConstPolygon3d> &trafficLights,
                                                 Ogre::SceneNode *parentNode) {
  // Create Manual Object, RefLines will be created as Manual Object using the
  // ogre_helper::drawLine helper function
  Ogre::ManualObject *trafficLightManualObject =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  trafficLightManualObject->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  for (auto &&trafficLight : trafficLights) {
    std::vector<Ogre::Vector3> line = Lanelet2Map::ogreLineFromLLetTrafficLight(trafficLight);
    drawArea(line, trafficLightManualObject, rend_opts_.colorTrafficLight);
  }
  if (trafficLightManualObject->getNumSections()) {
    parentNode->attachObject(trafficLightManualObject);
    // save ptr to manual object (needed later for deletion)
    objects_.push_back(std::make_pair(ObjectClassification::TRAFFICLIGHT, trafficLightManualObject));
  }
  trafficLightManualObject->end();
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L317
void Lanelet2Map::attachLaneletIdToSceneNode(const lanelet::ConstLanelet &lanelet, Ogre::SceneNode *parentNode) {
  // create child SceneNode. Only SceneNodes can be positioned.
  Ogre::SceneNode *childNode = parentNode->createChildSceneNode();

  rviz_rendering::MovableText *msg = new rviz_rendering::MovableText(std::to_string(lanelet.id()));
  msg->setCharacterHeight(rend_opts_.characterHeight);
  msg->setColor(rend_opts_.colorLaneletId);
  msg->setTextAlignment(rviz_rendering::MovableText::H_CENTER,
                        rviz_rendering::MovableText::V_ABOVE);  // Center horizontally and
                                                                // display above the node

  lanelet::ConstPoint3d text_pos(
      lanelet::utils::getId(), lanelet.centerline()[lanelet.centerline().size() / 2].x(),
      lanelet.centerline()[lanelet.centerline().size() / 2].y(),
      rend_opts_.threeD ? lanelet.centerline()[lanelet.centerline().size() / 2].z() - 0.2 : -0.2);

  Ogre::Vector3 trans = ogreVec3FromLLetPoint(text_pos);
  childNode->setPosition(trans);

  childNode->attachObject(msg);
  objects_.push_back(std::make_pair(ObjectClassification::LANELETID, msg));
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L405
std::vector<Ogre::Vector3> Lanelet2Map::ogreLineFromLLetLineString(const lanelet::ConstLineString3d &lineString) const {
  std::vector<Ogre::Vector3> line;
  for (lanelet::ConstPoint3d point : lineString) {
    line.push_back(ogreVec3FromLLetPoint(point));
  }
  return line;
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L413
std::vector<Ogre::Vector3> Lanelet2Map::ogreLineFromLLetPolygon(const lanelet::CompoundPolygon3d &polygon) const {
  // overloaded function to convert outer boundary of an area to an oger::line
  std::vector<Ogre::Vector3> line;
  for (lanelet::ConstPoint3d point : polygon) {
    line.push_back(ogreVec3FromLLetPoint(point));
  }
  return line;
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L422
std::vector<Ogre::Vector3> Lanelet2Map::ogreLineFromLLetTrafficLight(const lanelet::ConstPolygon3d &polygon) const {
  if (polygon.empty()) {
    return {};
  }

  // overloaded function to convert outer boundary of an area to a line
  std::vector<Ogre::Vector3> line;
  const lanelet::ConstPoint3d lowestPoint = *std::min_element(
      polygon.begin(), polygon.end(),
      [](const lanelet::ConstPoint3d &p1, const lanelet::ConstPoint3d &p2) { return p1.z() < p2.z(); });

  for (lanelet::ConstPoint3d point : polygon) {
    line.push_back(ogreVec3FromLLetTrafficLight(point, rend_opts_.trafficLightHeightAboveGround - lowestPoint.z()));
  }
  return line;
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L441
std::vector<Ogre::Vector3> Lanelet2Map::ogreLineFromLLetPts(const lanelet::ConstPoints3d &ptsVector) const {
  std::vector<Ogre::Vector3> line;
  for (lanelet::ConstPoint3d point : ptsVector) {
    line.push_back(ogreVec3FromLLetPoint(point));
  }
  return line;
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L450
Ogre::Vector3 Lanelet2Map::ogreVec3FromLLetPoint(const lanelet::ConstPoint3d point) const {
  using boost::numeric_cast;
  using boost::numeric::bad_numeric_cast;
  return Ogre::Vector3(numeric_cast<Ogre::Real>(point.x()), numeric_cast<Ogre::Real>(point.y()),
                       numeric_cast<Ogre::Real>(point.z()) * rend_opts_.threeD);
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L456
Ogre::Vector3 Lanelet2Map::ogreVec3FromLLetTrafficLight(const lanelet::ConstPoint3d point, const double zOffset) const {
  using boost::numeric_cast;
  using boost::numeric::bad_numeric_cast;
  // sets z-value to an appropriat height in a projected 2D map
  return Ogre::Vector3(numeric_cast<Ogre::Real>(point.x()), numeric_cast<Ogre::Real>(point.y()),
                       numeric_cast<Ogre::Real>(point.z() + zOffset));
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element_ogre_helper.hpp#L86
std::vector<Ogre::Vector3> Lanelet2Map::bufferSegment(const std::vector<Ogre::Vector3> &line, double buffer_length) {
  std::vector<Ogre::Vector3> buffered;
  buffered.reserve(line.size());
  assert(line.size() >= 2);
  for (auto it = line.begin(); it != line.end(); ++it) {
    auto normal = getNormal(it, line.begin(), line.end());
    buffered.push_back(*it + normal * buffer_length / 2);
  }
  for (auto it = line.rbegin(); it != line.rend(); ++it) {
    auto normal = getNormal(it, line.rbegin(), line.rend());
    buffered.push_back(*it + normal * buffer_length / 2);
  }
  return buffered;
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element_ogre_helper.hpp#L55
template <typename Iter>
Ogre::Vector3 Lanelet2Map::getNormal(Iter it, Iter begin, Iter end) {
  Ogre::Vector3 zero(0, 0, 0);
  // iterate backwards until direction vector is nonzero (in case of points on same position)
  auto dirBefore = zero;
  int i = 1;
  while (it != begin && dirBefore == zero) {
    dirBefore = *it - *std::prev(it, i);
    i++;
  }
  // same thing forwards
  auto dirAfter = zero;
  i = 1;
  while (std::next(it) != end && dirAfter == zero) {
    dirAfter = *std::next(it, i) - *it;
    i++;
  }
  // if no directions could be calculated, return 0 (only happens if there are no points or all points are identical)
  if (dirBefore == zero && dirAfter == zero) return zero;
  if (dirAfter == zero) dirAfter = dirBefore;
  if (dirBefore == zero) dirBefore = dirAfter;

  // calculate direction as mean from both line orientations
  auto dirCombined = dirAfter * (1 / dirAfter.length()) + dirBefore * (1 / dirBefore.length());
  dirCombined *= 1 / dirCombined.length();
  return Ogre::Vector3(dirCombined.y, -dirCombined.x, 0);  // rotate 90 degrees
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element_ogre_helper.hpp#L148
void Lanelet2Map::drawLine(const std::vector<Ogre::Vector3> &line, Ogre::ManualObject *obj, Ogre::ColourValue color,
                           double width) {
  if (width <= 0) return;
  if (line.size() < 2) return;
  auto buffered = bufferSegment(line, width);
  drawMonoPolygon(buffered, obj, color);
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element_ogre_helper.hpp#L141
void Lanelet2Map::drawArea(const std::vector<Ogre::Vector3> &line, Ogre::ManualObject *obj, Ogre::ColourValue color) {
  if (line.size() < 2) {
    return;
  }
  drawMonoPolygon(line, obj, color);
}

// https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element_ogre_helper.hpp#L101
/**
 * @brief drawMonoPolygon draws a monotone polygon in ogre
 * @param poly
 * @param color
 * @param objcv::norm(dir)
 */
void Lanelet2Map::drawMonoPolygon(const std::vector<Ogre::Vector3> &poly, Ogre::ManualObject *obj,
                                  Ogre::ColourValue color) {
  if (poly.size() < 3) return;

  auto itLeft = poly.begin();
  auto itRight = --poly.end();
  const auto startIndex = obj->getCurrentVertexCount();
  auto count = 0u;
  //    std::raise(SIGINT);
  //    obj->begin("osm_material", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  for (; itRight >= itLeft; ++itLeft, --itRight) {
    obj->position(*itLeft);
    obj->normal(0, 0, 1);
    obj->colour(color);
    if (count >= 2) {
      assert(obj->getCurrentVertexCount() > startIndex + count);
      obj->triangle(startIndex + count - 2, startIndex + count - 1, startIndex + count);
    }
    count++;
    obj->position(*itRight);
    obj->normal(0, 0, 1);
    obj->colour(color);
    if (count >= 2) {
      assert(obj->getCurrentVertexCount() > startIndex + count);
      // assert(obj->getCurrentVertexCount() < 65535);
      obj->triangle(startIndex + count, startIndex + count - 1, startIndex + count - 2);
    }
    count++;
  }
}

}  // namespace rviz_rendering
