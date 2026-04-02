// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include "lanelet2_rviz_display/lanelet2_map.hpp"

#include "lanelet2_map_helpers.hpp"

#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

namespace rviz_rendering {

int Lanelet2Map::manual_object_counter_{0};

Lanelet2Map::Lanelet2Map(Ogre::SceneManager* manager,
                         Ogre::SceneNode* parent_node,
                         Lanelet2Map::RenderingOptions rend_opts,
                         lanelet::LaneletMapConstPtr map_ptr)
    : rend_opts_(rend_opts), scene_manager_(manager), bsd_helpers_(std::make_unique<detail::Lanelet2MapHelpers>(*this)) {
  std::string ll_map_name = "Lanelet2Map";

  if (!parent_node) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  const std::string mat_surface_name = ll_map_name + "SurfaceMaterial";
  const std::string mat_line_name = ll_map_name + "LineMaterial";

  {
    auto res = Ogre::MaterialManager::getSingleton().createOrRetrieve(mat_surface_name, "rviz_rendering");
    auto mat = std::dynamic_pointer_cast<Ogre::Material>(res.first);
    if (mat) {
      material_surface_ = mat;
      material_surface_->setReceiveShadows(false);
      material_surface_->getTechnique(0)->setLightingEnabled(false);
      material_surface_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      material_surface_->getTechnique(0)->setDepthWriteEnabled(true);
      material_surface_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT + Ogre::TVC_DIFFUSE);
      material_surface_->getTechnique(0)->setCullingMode(Ogre::CULL_NONE);
      material_surface_->getTechnique(0)->getPass(0)->setDepthBias(1.0f, 1.0f);
    }
  }

  {
    auto res = Ogre::MaterialManager::getSingleton().createOrRetrieve(mat_line_name, "rviz_rendering");
    auto mat = std::dynamic_pointer_cast<Ogre::Material>(res.first);
    if (mat) {
      material_line_ = mat;
      material_line_->setReceiveShadows(false);
      material_line_->getTechnique(0)->setLightingEnabled(false);
      material_line_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      material_line_->getTechnique(0)->setDepthWriteEnabled(false);
      material_line_->getTechnique(0)->setDepthCheckEnabled(true);
      material_line_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT + Ogre::TVC_DIFFUSE);
      material_line_->getTechnique(0)->setCullingMode(Ogre::CULL_NONE);
      material_line_->getTechnique(0)->getPass(0)->setDepthBias(2.0f, 1.0f);
    }
  }

  create(map_ptr);
  updateVisibility(rend_opts_);
}

Lanelet2Map::~Lanelet2Map() {
  clearObjects();

  scene_node_->removeAndDestroyAllChildren();
  scene_node_->getParentSceneNode()->removeChild(scene_node_);
  scene_manager_->destroySceneNode(scene_node_);

  if (material_surface_) {
    material_surface_->unload();
  }
  if (material_line_) {
    material_line_->unload();
  }
}

void Lanelet2Map::clearObjects() {
  bsd_helpers_->clearObjects();
}

void Lanelet2Map::updateMap(Lanelet2Map::RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr) {
  rend_opts_ = rend_opts;
  clearObjects();
  create(map_ptr);
  updateVisibility(rend_opts_);
}

void Lanelet2Map::updateVisibility(const RenderingOptions& rend_opts) {
  rend_opts_ = rend_opts;
  updateVisibility(ObjectClassification::MAP, rend_opts_.renderLaneletLinestrings);
  updateVisibility(ObjectClassification::LANELETID, rend_opts_.renderLaneletIds);
  updateVisibility(ObjectClassification::AREA, rend_opts_.renderAreas);
  updateVisibility(ObjectClassification::PARKINGAREA, rend_opts_.renderParking);
  updateVisibility(ObjectClassification::SEPERATOR, rend_opts_.renderLaneletSeparators);
  updateVisibility(ObjectClassification::STOPLINE, rend_opts_.renderStopLines);
  updateVisibility(ObjectClassification::TRAFFICLIGHT, rend_opts_.renderTrafficLights);
  updateVisibility(ObjectClassification::LANEFILL, rend_opts_.renderLaneletFills);
  updateVisibility(ObjectClassification::SIDEWALK, rend_opts_.renderSidewalks);
  updateVisibility(ObjectClassification::CROSSWALK, rend_opts_.renderCrosswalks);
}

void Lanelet2Map::updateVisibility(ObjectClassification classification, bool visible) {
  for (auto object : objects_) {
    if (object.first == classification) {
      object.second->setVisible(visible);
    }
  }
}

void Lanelet2Map::create(lanelet::LaneletMapConstPtr map_ptr) {
  Ogre::ManualObject* map_manual_object =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject* separator_manual_object =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject* area_manual_object =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject* parking_manual_object =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject* lane_fill_manual_object =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject* sidewalk_manual_object =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));
  Ogre::ManualObject* crosswalk_manual_object =
      scene_manager_->createManualObject("llet_object_" + std::to_string(manual_object_counter_++));

  if (map_ptr == nullptr) {
    scene_manager_->destroyManualObject(map_manual_object);
    scene_manager_->destroyManualObject(separator_manual_object);
    scene_manager_->destroyManualObject(area_manual_object);
    scene_manager_->destroyManualObject(parking_manual_object);
    scene_manager_->destroyManualObject(lane_fill_manual_object);
    scene_manager_->destroyManualObject(sidewalk_manual_object);
    scene_manager_->destroyManualObject(crosswalk_manual_object);
    return;
  }

  map_manual_object->begin(material_line_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  separator_manual_object->begin(material_line_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  area_manual_object->begin(material_surface_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  parking_manual_object->begin(material_surface_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  lane_fill_manual_object->begin(material_surface_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  sidewalk_manual_object->begin(material_surface_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  crosswalk_manual_object->begin(material_surface_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");

  const uint8_t q_surfaces = Ogre::RENDER_QUEUE_MAIN;
  const uint8_t q_lines = Ogre::RENDER_QUEUE_MAIN + 5;
  lane_fill_manual_object->setRenderQueueGroup(q_surfaces);
  area_manual_object->setRenderQueueGroup(q_surfaces);
  parking_manual_object->setRenderQueueGroup(q_surfaces);
  sidewalk_manual_object->setRenderQueueGroup(q_surfaces);
  crosswalk_manual_object->setRenderQueueGroup(q_surfaces);
  map_manual_object->setRenderQueueGroup(q_lines);
  separator_manual_object->setRenderQueueGroup(q_lines);

  for (const lanelet::ConstLanelet& lanelet : map_ptr->laneletLayer) {
    bsd_helpers_->addLaneletToManualObject(lanelet, map_manual_object);
    bsd_helpers_->addSeperatorToManualObject(lanelet, separator_manual_object);
    if (rend_opts_.renderLaneletFills) {
      bsd_helpers_->addLaneFillToManualObject(lanelet, lane_fill_manual_object);
    }
    bsd_helpers_->addRegulatoryElements(lanelet, scene_node_);
    bsd_helpers_->attachLaneletIdToSceneNode(lanelet, scene_node_);
  }

  for (const lanelet::ConstArea& area : map_ptr->areaLayer) {
    auto attributes = area.attributes();
    const auto subtype = attributes[lanelet::AttributeName::Subtype];
    if (subtype == lanelet::AttributeValueString::Parking) {
      bsd_helpers_->addParkingAreaToManualObject(area, parking_manual_object);
    } else if (rend_opts_.renderCrosswalks && (subtype == "crosswalk")) {
      bsd_helpers_->addCrosswalkToManualObject(area, crosswalk_manual_object);
    } else if (rend_opts_.renderSidewalks &&
               (subtype == "sidewalk" || subtype == "walkway" || subtype == "footway")) {
      bsd_helpers_->addSidewalkToManualObject(area, sidewalk_manual_object);
    } else {
      bsd_helpers_->addAreaToManualObject(area, area_manual_object);
    }
  }

  map_manual_object->end();
  separator_manual_object->end();
  area_manual_object->end();
  parking_manual_object->end();
  lane_fill_manual_object->end();
  sidewalk_manual_object->end();
  crosswalk_manual_object->end();

  auto attach_if_populated = [this](Ogre::ManualObject* manual_object, ObjectClassification classification) {
    if (manual_object->getNumSections()) {
      scene_node_->attachObject(manual_object);
      objects_.push_back(std::make_pair(classification, manual_object));
    } else {
      scene_manager_->destroyManualObject(manual_object);
    }
  };

  attach_if_populated(map_manual_object, ObjectClassification::MAP);
  attach_if_populated(separator_manual_object, ObjectClassification::SEPERATOR);
  attach_if_populated(area_manual_object, ObjectClassification::AREA);
  attach_if_populated(parking_manual_object, ObjectClassification::PARKINGAREA);
  attach_if_populated(lane_fill_manual_object, ObjectClassification::LANEFILL);
  attach_if_populated(sidewalk_manual_object, ObjectClassification::SIDEWALK);
  attach_if_populated(crosswalk_manual_object, ObjectClassification::CROSSWALK);
}

}  // namespace rviz_rendering
