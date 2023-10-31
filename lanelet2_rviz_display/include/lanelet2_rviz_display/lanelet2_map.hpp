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

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <boost/numeric/conversion/cast.hpp>


namespace rviz_rendering
{
    /**
     * \class Lanelet2Map
     * \brief Displays a Lanelet2Map
     *
     * Displays a Lanelet2Map.
     */
    class Lanelet2Map
    {
        public:

            struct RenderingOptions {
                // Line-Strings
                bool renderLaneletLinestrings = true;
                double linestringWidth = 0.1;
                Ogre::ColourValue colorLeft{Ogre::ColourValue(1.0, 1.0, 1.0, 1.0)};         // white
                Ogre::ColourValue colorRight{Ogre::ColourValue(1.0, 1.0, 1.0, 1.0)};        // white

                // Separator
                bool renderLaneletSeparators = false;
                Ogre::ColourValue colorSeperator{Ogre::ColourValue(0.1, 0.1, 0.9, 1.0)};    // blue
                double seperatorWidth = 0.2;

                // StopLines
                Ogre::ColourValue colorStopLine{Ogre::ColourValue(1.0, 0.1, 0.1, 1.0)};     // red
                double stopLineWidth = 0.2;
                
                // ID's
                double characterHeight = 1.0;

                // 3D
                bool threeD = true;
            };

            Lanelet2Map(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node, Lanelet2Map::RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr);
            ~Lanelet2Map();

            /**
             * \brief Get the Ogre scene node associated with this ll2-map
             *
             * @return The Ogre scene node associated with this ll2-map
             */
            Ogre::SceneNode * getSceneNode() {return scene_node_;}

            void updateMap(Lanelet2Map::RenderingOptions rend_opts, lanelet::LaneletMapConstPtr map_ptr);

        private:

            Lanelet2Map::RenderingOptions rend_opts_; // Rendering options for this ll2-map

            Ogre::SceneManager *scene_manager_;
            Ogre::SceneNode *scene_node_;           // The scene node that this ll2-map is attached to
            Ogre::ManualObject *manual_object_;     // The manual object used to draw the ll2-map

            Ogre::MaterialPtr material_;

            void create(lanelet::LaneletMapConstPtr map_ptr);
            
            void addLaneletToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);
            void drawLine(const std::vector<Ogre::Vector3>& line, Ogre::ManualObject* obj, Ogre::ColourValue color = Ogre::ColourValue::White, double width = 0.1);
            std::vector<Ogre::Vector3> bufferSegment(const std::vector<Ogre::Vector3>& line, double buffer_length);
            template <typename Iter> Ogre::Vector3 getNormal(Iter it, Iter begin, Iter end);
            void drawMonoPolygon(const std::vector<Ogre::Vector3>& poly, Ogre::ManualObject* obj, Ogre::ColourValue color = Ogre::ColourValue::White);
            std::vector<Ogre::Vector3> ogreLineFromLLetLineString(const lanelet::ConstLineString3d& lineString) const;
            Ogre::Vector3 ogreVec3FromLLetPoint(const lanelet::ConstPoint3d point) const;

            void addSeperatorToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);
            std::vector<Ogre::Vector3> ogreLineFromLLetPts(const lanelet::ConstPoints3d& ptsVector) const;


    };

}  // namespace rviz_rendering

#endif  // LANELET2_RVIZ_DISPLAY__LANELET2_MAP_HPP_