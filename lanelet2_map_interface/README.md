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
                ll2_interface_ = std::make_unique<LL2MapInterface>(*this, ll2_map_server_name_);
            }
        private:
            std::string ll2_map_server_name_ = "lanelet2_map_server";
            std::unique_ptr<LL2MapInterface> ll2_interface_;
    }
    ```
1. Access the current Lanelet2 map via the interface. A browsable [API documentation](https://openads-project.github.io/lanelet2_map_server) is available.
    ```c++
    if(ll2_interface_->map_loaded_) {
        lanelet::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
        lanelet::LaneletMapPtr nonconst_map = ll2_interface_->getNonConstMapPtr();
        std::shared_ptr<lanelet::Projector> proj = ll2_interface_->getProjectorPtr();
        std::string map_frame_id = ll2_interface_->map_frame_id
        if(ll2_interface_->update_pending_) {
            ...
            // map provided by the server has changed
            // update local variables
            map = ll2_interface_->getMapPtr();
            // ...
            ll2_interface_->update_pending_ = false;
        }
    }
    ```
