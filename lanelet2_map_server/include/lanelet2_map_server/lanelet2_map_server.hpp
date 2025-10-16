#include "rclcpp/rclcpp.hpp"

#include <string.h>
#include <fstream>
#include <limits>
#include <vector>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_projection/UTM.h>

#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

struct Lanelet2MapMeta {
    std::string map_path;
    double min_lat = 0.0;
    double min_lon = 0.0;
    double max_lat = 0.0;
    double max_lon = 0.0;
    double diagonal_length = -1.0;
};

class LL2MapServer : public rclcpp::Node
{
    public:
        LL2MapServer();

    private:

        void declareParameters();

        void loadParameters();

        void setup();
        void find_available_maps(const std::string& directory, std::vector<Lanelet2MapMeta>& maps) const;
        void derive_map_bounds(Lanelet2MapMeta& map_meta) const;

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

        void loadMapContents();

        bool map_sanity_check(std::string map_filename, double origin_lat, double origin_lon);
        void pub_tf();
        void derive_utm_zone(const double latitude, const double longitude, int& zone, bool& northp);
        void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void automaticMapUpdateTimerCallback();

    private:
        OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

        rclcpp::TimerBase::SharedPtr one_shot_timer_;
        rclcpp::TimerBase::SharedPtr automatic_map_timer_;
        
        bool use_automatic_map_selection_ = true;
        bool use_manual_origin_ = false;

        std::string map_directory_ = "/data/maps/locations";
        std::vector<Lanelet2MapMeta> available_maps_;
        std::string map_filepath_;
        std::string map_frame_id_ = "map";
        std::string map_contents_;
        double origin_lat_, origin_lon_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_subscription_;
        double current_latitude_ = std::numeric_limits<double>::quiet_NaN();
        double current_longitude_ = std::numeric_limits<double>::quiet_NaN();
        bool gps_fix_received_ = false;
};
