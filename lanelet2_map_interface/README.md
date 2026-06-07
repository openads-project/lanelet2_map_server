# `lanelet2_map_interface`

Library to receive Lanelet2 maps from lanelet2_map_server

The `lanelet2_map_interface` is a library to be included into other nodes that rely on Lanelet2 map information. By including the interface, nodes can access one commonly loaded map distributed by the [lanelet2_map_server](../lanelet2_map_server).

### Integration

The library can be integrated into nodes that require access to the current Lanelet2 map by following these steps. One exemplary node integrating the interface is [lanelet2_route_planning](https://github.com/openads-project/lanelet2_route_planning).

1. Add the `lanelet2_map_interface` as a dependency in `package.xml`, `CMakeLists.txt` (and `.repos`).
1. Integrate the interface into the node.

    ```c++
    #include "rclcpp/rclcpp.hpp"
    ...
    #include "lanelet2_map_interface/lanelet2_map_interface.hpp"
    ...
    class MyNode : public rclcpp::Node {
        public:
            MyNode() {
                lanelet2_map_interface_ = std::make_unique<Lanelet2MapInterface>(*this, lanelet2_map_server_name_);
            }
        private:
            std::string lanelet2_map_server_name_ = "lanelet2_map_server";
            std::unique_ptr<Lanelet2MapInterface> lanelet2_map_interface_;
    }
    ```
1. Access the current Lanelet2 map via the interface. A browsable [API documentation](https://openads-project.github.io/lanelet2_map_server) is available.
    ```c++
    if(lanelet2_map_interface_->map_loaded_) {
        lanelet::LaneletMapConstPtr map = lanelet2_map_interface_->getMapPtr();
        lanelet::LaneletMapPtr nonconst_map = lanelet2_map_interface_->getNonConstMapPtr();
        std::shared_ptr<lanelet::Projector> proj = lanelet2_map_interface_->getProjectorPtr();
        std::string map_frame_id = lanelet2_map_interface_->map_frame_id
        if(lanelet2_map_interface_->update_pending_) {
            ...
            // map provided by the server has changed
            // update local variables
            map = lanelet2_map_interface_->getMapPtr();
            // ...
            lanelet2_map_interface_->update_pending_ = false;
        }
    }
    ```
