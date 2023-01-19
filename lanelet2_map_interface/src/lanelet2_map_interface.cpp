#include "lanelet2_map_interface/lanelet2_map_interface.hpp"

LL2MapInterface::LL2MapInterface(rclcpp::Node::SharedPtr parent_node, std::string map_server_name)
{
    parent_node_ = parent_node;
    map_server_name_ = map_server_name;
    RCLCPP_INFO_STREAM(parent_node_->get_logger(), "This is the Lanelet2-Interface of " << parent_node_->get_name());
    // We need a callback group here since the service is called within a callback
    // https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html#examples
    client_callback_group_ = parent_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    parameter_client_ = parent_node_->create_client<lanelet2_map_manager_ifs::srv::ProvideMapParams>(map_server_name_+"/provide_map_parameters", rmw_qos_profile_services_default, client_callback_group_);
    reload_sub_ = parent_node_->create_subscription<lanelet2_map_manager_ifs::msg::MapChange>(map_server_name_+"/map_changed", 1, std::bind(&LL2MapInterface::mapChangeCallback, this, std::placeholders::_1));
    startup_timer_ = parent_node_->create_wall_timer(1.0s, std::bind(&LL2MapInterface::startupTimerCallback, this));
}

lanelet::LaneletMapPtr LL2MapInterface::getNonConstMapPtr()
{
    if(!map_loaded_)
    {
        RCLCPP_ERROR(parent_node_->get_logger(), "Lanelet2-Map is currently not loaded. Returning nullptr!");
        return nullptr;
    }
    else
    {
        return mapPtr_;
    }
}

lanelet::LaneletMapConstPtr LL2MapInterface::getMapPtr()
{
    return getNonConstMapPtr();
}

std::shared_ptr<lanelet::Projector> LL2MapInterface::getProjectorPtr()
{
    if(!map_loaded_)
    {
        RCLCPP_ERROR(parent_node_->get_logger(), "Lanelet2-Projector is currently not initialized. Returning nullptr!");
        return nullptr;
    }
    else
    {
        return utmProjectorPtr_;
    }
}


void LL2MapInterface::startupTimerCallback()
{
    RCLCPP_INFO(parent_node_->get_logger(), "Try to initialize Lanelet2-Map!");
    if(loadMap())
    {
        RCLCPP_INFO(parent_node_->get_logger(), "Initialized Lanelet2-Map successfully!");
        startup_timer_->cancel();
    }
}

void LL2MapInterface::mapChangeCallback(const lanelet2_map_manager_ifs::msg::MapChange::SharedPtr msg)
{
    RCLCPP_INFO(parent_node_->get_logger(), "Lanelet2-Map has changed, reloading!");
    if(!loadMap())
    {
        RCLCPP_ERROR(parent_node_->get_logger(), "Reloading of Lanelet2-Map was not successful!");
    }
}

bool LL2MapInterface::loadMap()
{
    // Get parameters
    auto request = std::make_shared<lanelet2_map_manager_ifs::srv::ProvideMapParams::Request>();
    request->requesting_node_name=parent_node_->get_name();
    if(!parameter_client_->wait_for_service(0.1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(parent_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_WARN_STREAM(parent_node_->get_logger(), "Service "+ map_server_name_ +"/provide_map_parameters is not available, waiting...");
        return false;
    }
    auto result = parameter_client_->async_send_request(request);
    std::future_status status = result.wait_for(0.1s);  // timeout to guarantee a graceful finish
    // Wait for the result.
    if (status == std::future_status::ready)
    {
        lanelet2_map_manager_ifs::srv::ProvideMapParams::Response res = *result.get();
        utmProjectorPtr_ = std::make_shared<lanelet::projection::UtmProjector>(lanelet::Origin({res.origin_lat, res.origin_lon}));
        mapPtr_ = lanelet::load(res.map_filename, *utmProjectorPtr_);
        map_frame_id_ = res.map_frame_id;
        map_loaded_=true;
        RCLCPP_INFO_STREAM(parent_node_->get_logger(), "Loaded "+ res.map_filename +" succesfully!");
        return true;
    }
    else
    {
        map_loaded_=false;
        RCLCPP_ERROR_STREAM(parent_node_->get_logger(), "Failed to call service "+ map_server_name_ +"/provide_map_parameters");
        return false;
    }
}