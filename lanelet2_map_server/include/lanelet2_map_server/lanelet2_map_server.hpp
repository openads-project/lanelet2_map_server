#include "rclcpp/rclcpp.hpp"

#include <string.h>

#include "lanelet2_map_manager_srvs/srv/change_map_params.hpp"
#include "lanelet2_map_manager_srvs/srv/provide_map_params.hpp"

#include "lanelet2_map_manager_msgs/msg/map_change.hpp"

class LL2MapServer : public rclcpp::Node
{
    public:
        LL2MapServer();
        
    private:

        void change_params(const std::shared_ptr<lanelet2_map_manager_srvs::srv::ChangeMapParams::Request> request, std::shared_ptr<lanelet2_map_manager_srvs::srv::ChangeMapParams::Response> response);
        void provide_params(const std::shared_ptr<lanelet2_map_manager_srvs::srv::ProvideMapParams::Request> request, std::shared_ptr<lanelet2_map_manager_srvs::srv::ProvideMapParams::Response> response);

        rclcpp::Service<lanelet2_map_manager_srvs::srv::ChangeMapParams>::SharedPtr change_map_srv_;
        rclcpp::Service<lanelet2_map_manager_srvs::srv::ProvideMapParams>::SharedPtr provide_map_srv_;

        rclcpp::Publisher<lanelet2_map_manager_msgs::msg::MapChange>::SharedPtr map_change_pub_;

        std::string map_filename_, map_frame_id_;
        double origin_lat_, origin_lon_;
        bool params_set_=false;

};