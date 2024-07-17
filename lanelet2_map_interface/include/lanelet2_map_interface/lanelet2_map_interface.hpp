#include "rclcpp/rclcpp.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string.h>
#include <cstdlib>

using namespace std::chrono_literals;
class LL2MapInterface
{
    public:
        LL2MapInterface(rclcpp::Node& parent_node, std::string map_server_name);
        lanelet::LaneletMapConstPtr getMapPtr();
        lanelet::LaneletMapPtr getNonConstMapPtr();
        std::shared_ptr<lanelet::Projector> getProjectorPtr();
        bool map_loaded_=false;
        bool update_pending_=false; // Flag indicating if the client node should update map
        std::string map_frame_id_;

    private:
        rclcpp::Node& parent_node_;
        std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
        std::shared_ptr<rclcpp::ParameterEventHandler> parameter_sub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> frame_id_callback_handle_, contents_callback_handle_, origin_lat_callback_handle_, origin_lon_callback_handle_;
        rclcpp::TimerBase::SharedPtr startup_timer_;

        std::string map_filepath_ = std::string(getenv("HOME")) + "/.ros/lanelet2_map_interface/map.osm";

        std::vector<rclcpp::Parameter> map_params_;
        std::string map_contents_;
        double origin_lat_=91.0, origin_lon_=181.0; // init to invalid values

        lanelet::LaneletMapPtr mapPtr_;
        std::shared_ptr<lanelet::Projector> utmProjectorPtr_;

        std::string map_server_name_;
        bool params_declared_ = false;

        void updateMapParam(rclcpp::Parameter param);
        bool loadMap();
        bool validateParams();

        void findMapServer();
        void updateParamsCallback(const rclcpp::Parameter & p);
        void serviceParamsCallback(std::shared_future<std::vector<rclcpp::Parameter>> future);
};