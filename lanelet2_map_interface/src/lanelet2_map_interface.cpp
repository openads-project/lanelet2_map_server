#include "lanelet2_map_interface/lanelet2_map_interface.hpp"

LL2MapInterface::LL2MapInterface(rclcpp::Node::SharedPtr parent_node, std::string map_server_name)
{
    parent_node_ = parent_node;
    map_server_name_ = map_server_name;
    RCLCPP_INFO_STREAM(parent_node_->get_logger(), "This is the Lanelet2-Interface of " << parent_node_->get_name());
    
    // Initialize parameter client and event handler
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(parent_node_, map_server_name);
    parameter_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(parent_node_);
    
    while (!parameter_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(parent_node_->get_logger(), "Interrupted while waiting for the parameter-service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(parent_node_->get_logger(), "LL2-Map-Server Parameter-service is not available, waiting...");
    }

    auto update_callback = [this](const rclcpp::Parameter & p) 
    {
        RCLCPP_INFO_STREAM(
        parent_node_->get_logger(),
        "Received an update to parameter " << p.get_name() << "! \n Reloading lanelet2-map!");
        updateMapParam(p);
        loadMap();
    };
    
    filepath_callback_handle_ = parameter_sub_->add_parameter_callback("map_filepath", update_callback, map_server_name_);
    frame_id_callback_handle_ = parameter_sub_->add_parameter_callback("map_frame_id", update_callback, map_server_name_);
    origin_lat_callback_handle_ = parameter_sub_->add_parameter_callback("origin_lat", update_callback, map_server_name_);
    origin_lon_callback_handle_ = parameter_sub_->add_parameter_callback("origin_lon", update_callback, map_server_name_);
    
    map_params_ = parameter_client_->get_parameters({"map_filepath", "map_frame_id", "origin_lat", "origin_lon"});
    for (auto & parameter : map_params_)
    {
        updateMapParam(parameter);
    }
    loadMap();
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

void LL2MapInterface::updateMapParam(rclcpp::Parameter param)
{
    if(param.get_name()=="map_frame_id")
    {
        map_frame_id_ = param.value_to_string();
    }
    if(param.get_name()=="map_filepath")
    {
        map_filepath_ = param.value_to_string();
    }
    if(param.get_name()=="origin_lat")
    {
        origin_lat_ = param.as_double();
    }
    if(param.get_name()=="origin_lon")
    {
        origin_lat_ = param.as_double();
    }
}

bool LL2MapInterface::loadMap()
{
    utmProjectorPtr_ = std::make_shared<lanelet::projection::UtmProjector>(lanelet::Origin({origin_lat_, origin_lon_}));
    mapPtr_ = lanelet::load(map_filepath_, *utmProjectorPtr_);
    map_loaded_=true;
    RCLCPP_INFO_STREAM(parent_node_->get_logger(), "Loaded "+ map_filepath_ +" succesfully!");
    return true;
}