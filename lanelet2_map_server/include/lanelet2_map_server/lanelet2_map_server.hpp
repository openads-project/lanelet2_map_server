#include "rclcpp/rclcpp.hpp"

#include <string.h>

#include "lanelet2_map_server_ifs/srv/change_map_params.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_projection/UTM.h>

class LL2MapServer : public rclcpp::Node
{
    public:
        LL2MapServer();
        
    private:

        void change_params(const std::shared_ptr<lanelet2_map_server_ifs::srv::ChangeMapParams::Request> request, std::shared_ptr<lanelet2_map_server_ifs::srv::ChangeMapParams::Response> response);
        bool map_sanity_check(std::string map_filename, double origin_lat, double origin_lon);

        rclcpp::Service<lanelet2_map_server_ifs::srv::ChangeMapParams>::SharedPtr change_map_srv_;

        std::string map_filename_, map_frame_id_;
        double origin_lat_, origin_lon_;

        bool init_=false;

};