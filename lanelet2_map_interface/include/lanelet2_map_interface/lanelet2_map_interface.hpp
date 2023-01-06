#include "rclcpp/rclcpp.hpp"

#include "lanelet2_map_manager_ifs/srv/provide_map_params.hpp"
#include "lanelet2_map_manager_ifs/msg/map_change.hpp"

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
        bool map_loaded_=false;
        std::string map_frame_id_;
        
    private:
        rclcpp::Node::SharedPtr parent_node_;
        rclcpp::Client<lanelet2_map_manager_ifs::srv::ProvideMapParams>::SharedPtr parameter_client_;
        rclcpp::CallbackGroup::SharedPtr client_callback_group_;
        rclcpp::Subscription<lanelet2_map_manager_ifs::msg::MapChange>::SharedPtr reload_sub_;
        rclcpp::TimerBase::SharedPtr startup_timer_;

        lanelet::LaneletMapPtr mapPtr_;
        std::shared_ptr<lanelet::Projector> utmProjectorPtr_;
        
        std::string map_server_name_;

        void mapChangeCallback(const lanelet2_map_manager_ifs::msg::MapChange::SharedPtr msg);
        void startupTimerCallback();
        bool loadMap();
};