#include "rclcpp/rclcpp.hpp"

#include <string>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <tuple>
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

template <typename C> struct is_vector : std::false_type {};
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;

class LL2MapServer : public rclcpp::Node
{
    public:
        LL2MapServer();

    private:

        /**
         * @brief Declares and loads a ROS parameter
         *
         * @param name name
         * @param param parameter variable to load into
         * @param description description
         * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
         * @param is_required whether failure to load parameter will stop node
         * @param read_only set parameter to read-only
         * @param from_value parameter range minimum
         * @param to_value parameter range maximum
         * @param step_value parameter range step
         * @param additional_constraints additional constraints description
         */
        template <typename T>
        void declareAndLoadParameter(const std::string &name,
                                    T &param,
                                    const std::string &description,
                                    const bool add_to_auto_reconfigurable_params = true,
                                    const bool is_required = false,
                                    const bool read_only = false,
                                    const std::optional<double> &from_value = std::nullopt,
                                    const std::optional<double> &to_value = std::nullopt,
                                    const std::optional<double> &step_value = std::nullopt,
                                    const std::string &additional_constraints = "");

        /**
         * @brief Handles reconfiguration when a parameter value is changed
         *
         * @param parameters parameters
         * @return parameter change result
         */
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

        void loadMapContents();
        void unsetMapParameters();
        void updateMapParameters();

        void setup();
        void find_available_maps(const std::string& directory, std::vector<Lanelet2MapMeta>& maps) const;
        void derive_map_meta(Lanelet2MapMeta& map_meta) const;

        bool map_sanity_check(std::string map_filename, double origin_lat, double origin_lon) const;
        void pub_tf() const;
        void derive_utm_zone(const double latitude, const double longitude, int& zone, bool& northp) const;
        void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void automaticMapUpdateTimerCallback();

    private:
        /**
         * @brief Auto-reconfigurable parameters for dynamic reconfiguration
         */
        std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;

        /**
         * @brief Callback handle for dynamic parameter reconfiguration
         */
        OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

        rclcpp::TimerBase::SharedPtr one_shot_timer_;
        rclcpp::TimerBase::SharedPtr automatic_map_timer_;
        
        bool use_automatic_map_selection_ = true;

        std::string map_directory_ = "/data/maps/default-maps";
        std::vector<Lanelet2MapMeta> available_maps_;
        std::string map_filepath_;
        std::string map_frame_id_ = "map";
        std::string map_contents_;
        double origin_lat_ = 0.0;
        double origin_lon_ = 0.0;
        bool origin_lat_initialized_ = false;
        bool origin_lon_initialized_ = false;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_subscription_;
        double current_latitude_ = std::numeric_limits<double>::quiet_NaN();
        double current_longitude_ = std::numeric_limits<double>::quiet_NaN();
        bool gps_fix_received_ = false;
};
