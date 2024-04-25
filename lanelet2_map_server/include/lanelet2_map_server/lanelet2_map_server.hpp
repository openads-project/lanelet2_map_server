#include "rclcpp/rclcpp.hpp"

#include <string.h>
#include <fstream>

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

        void declareParameters();

        void loadParameters();

        void setup();

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

        void loadMapContents();

        bool map_sanity_check(std::string map_filename, std::string map_frame_id, double origin_lat, double origin_lon);
        void pub_tf();
        void derive_utm_zone(const double latitude, const double longitude, int& zone, bool& northp);

    private:
        OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

        rclcpp::TimerBase::SharedPtr one_shot_timer_;

        std::string map_filepath_;
        std::string map_frame_id_ = "map";
        std::string map_contents_;
        double origin_lat_, origin_lon_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};