#include "rclcpp/rclcpp.hpp"

#include "lanelet2_map_manager_srvs/srv/provide_map_params.hpp"
#include "lanelet2_map_manager_msgs/msg/map_change.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <string.h>

using namespace std::chrono_literals;
class LL2MapInterface
{
    public:
        LL2MapInterface(rclcpp::Node::SharedPtr parent_node, std::string map_server_name);
        lanelet::LaneletMapConstPtr getMapPtr();
        lanelet::LaneletMapPtr getNonConstMapPtr();
        std::shared_ptr<lanelet::Projector> getProjectorPtr();
        
    private:
        rclcpp::Node::SharedPtr parent_node_;
        rclcpp::Client<lanelet2_map_manager_srvs::srv::ProvideMapParams>::SharedPtr parameter_client_;
        rclcpp::Subscription<lanelet2_map_manager_msgs::msg::MapChange>::SharedPtr reload_sub_;

        lanelet::LaneletMapPtr mapPtr_;
        std::shared_ptr<lanelet::Projector> utmProjectorPtr_;
        
        bool map_loaded_=false;
        std::string map_server_name_;

        void mapChangeCallback(const lanelet2_map_manager_msgs::msg::MapChange::SharedPtr msg);
        void loadMap();
};