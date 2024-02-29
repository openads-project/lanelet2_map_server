#include "rclcpp/rclcpp.hpp"

#include <string.h>

#include "lanelet2_map_server_interfaces/srv/change_map_params.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_projection/UTM.h>

#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class LL2MapServer : public rclcpp::Node
{
    public:
        LL2MapServer();
        
    private:

        void change_params(const std::shared_ptr<lanelet2_map_server_interfaces::srv::ChangeMapParams::Request> request, std::shared_ptr<lanelet2_map_server_interfaces::srv::ChangeMapParams::Response> response);
        bool map_sanity_check(std::string map_filename, std::string map_frame_id, double origin_lat, double origin_lon);
        void pub_tf();
        void derive_utm_zone(const double latitude, const double longitude, int& zone, bool& northp);

        rclcpp::Service<lanelet2_map_server_interfaces::srv::ChangeMapParams>::SharedPtr change_map_srv_;

        std::string map_filename_, map_frame_id_;
        double origin_lat_, origin_lon_;

        bool init_=false;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};