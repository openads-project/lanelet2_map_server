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

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

namespace rviz_rendering
{
    
    Lanelet2Map::Lanelet2Map(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node, Lanelet2Map::RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr):
    scene_manager_(manager),
    rend_opts_(rend_opts)
    {
        std::string ll_map_name = "Lanelet2Map";
        manual_object_ = scene_manager_->createManualObject(ll_map_name);

        if (!parent_node) {
            parent_node = scene_manager_->getRootSceneNode();
        }

        scene_node_ = parent_node->createChildSceneNode();
        scene_node_->attachObject(manual_object_);

        std::string map_material_name = ll_map_name + "Material";

        auto retrieve_result = Ogre::MaterialManager::getSingleton().createOrRetrieve(map_material_name, "rviz_rendering");
        auto material = std::dynamic_pointer_cast<Ogre::Material>(retrieve_result.first);
        if(material)
        {
            material_ = material;
            material_->setReceiveShadows(false);
            material_->getTechnique(0)->setLightingEnabled(false);
            material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
            material_->getTechnique(0)->setDepthWriteEnabled(true);
            material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT + Ogre::TVC_DIFFUSE);
            material_->getTechnique(0)->setCullingMode(Ogre::CULL_NONE); // No culling
            create(map_ptr);
        }
    }

    Lanelet2Map::~Lanelet2Map()
    {
        scene_manager_->destroySceneNode(scene_node_);
        scene_manager_->destroyManualObject(manual_object_);

        material_->unload();
    }

    void Lanelet2Map::updateMap(Lanelet2Map::RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr)
    {
        rend_opts_=rend_opts;
        create(map_ptr);
    }

    void Lanelet2Map::create(lanelet::LaneletMapConstPtr map_ptr)
    {
        manual_object_->clear();
        if(map_ptr != nullptr)
        {
            manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
            // Iterate through all lanelets in map-graph
            for (const lanelet::ConstLanelet& lanelet : map_ptr->laneletLayer) 
            {   
                if(rend_opts_.renderLaneletLinestrings)
                {
                    addLaneletToManualObject(lanelet, manual_object_);
                }
                if(rend_opts_.renderLaneletSeparators)
                {
                    addSeperatorToManualObject(lanelet, manual_object_);
                }
            }
            manual_object_->end();
        }
    }

    // https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L339
    void Lanelet2Map::addLaneletToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual) {
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

    // https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element_ogre_helper.hpp#L148
    void Lanelet2Map::drawLine(const std::vector<Ogre::Vector3>& line, Ogre::ManualObject* obj, Ogre::ColourValue color, double width) {
        if (width <= 0)
            return;
        if (line.size() < 2)
            return;
        auto buffered = bufferSegment(line, width);
        drawMonoPolygon(buffered, obj, color);
    }

    // https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element_ogre_helper.hpp#L86
    std::vector<Ogre::Vector3> Lanelet2Map::bufferSegment(const std::vector<Ogre::Vector3>& line, double buffer_length) {
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
        if (dirBefore == zero && dirAfter == zero)
            return zero;
        if (dirAfter == zero)
            dirAfter = dirBefore;
        if (dirBefore == zero)
            dirBefore = dirAfter;

        // calculate direction as mean from both line orientations
        auto dirCombined = dirAfter * (1 / dirAfter.length()) + dirBefore * (1 / dirBefore.length());
        dirCombined *= 1 / dirCombined.length();
        return Ogre::Vector3(dirCombined.y, -dirCombined.x, 0); // rotate 90 degrees
    }

    // https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element_ogre_helper.hpp#L101
    /**
     * @brief drawMonoPolygon draws a monotone polygon in ogre
     * @param poly
     * @param color
     * @param objcv::norm(dir)
     */
    void Lanelet2Map::drawMonoPolygon(const std::vector<Ogre::Vector3>& poly,
                                        Ogre::ManualObject* obj,
                                        Ogre::ColourValue color)
    {
        if (poly.size() < 3)
            return;

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

    // https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L405
    std::vector<Ogre::Vector3> Lanelet2Map::ogreLineFromLLetLineString(const lanelet::ConstLineString3d& lineString) const {
        std::vector<Ogre::Vector3> line;
        for (lanelet::ConstPoint3d point : lineString) {
            line.push_back(ogreVec3FromLLetPoint(point));
        }
        return line;
    }

    // https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L450
    Ogre::Vector3 Lanelet2Map::ogreVec3FromLLetPoint(const lanelet::ConstPoint3d point) const {
        using boost::numeric_cast;
        using boost::numeric::bad_numeric_cast;
        return Ogre::Vector3(numeric_cast<Ogre::Real>(point.x()), numeric_cast<Ogre::Real>(point.y()), numeric_cast<Ogre::Real>(point.z()) * rend_opts_.threeD);
    }

    // https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L384
    void Lanelet2Map::addSeperatorToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual) {
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
    
    // https://github.com/coincar-sim/lanelet_rviz_plugin_ros/blob/9a36cb891ef0d175e0a893526980ea515712e1b9/src/map_element.cpp#L441
    std::vector<Ogre::Vector3> Lanelet2Map::ogreLineFromLLetPts(const lanelet::ConstPoints3d& ptsVector) const {
        std::vector<Ogre::Vector3> line;
        for (lanelet::ConstPoint3d point : ptsVector) {
            line.push_back(ogreVec3FromLLetPoint(point));
        }
        return line;
    }    

} // namespace rviz_rendering
