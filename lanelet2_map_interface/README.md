# `lanelet2_map_interface`

The `lanelet2_map_interface` is a library that should be included into various ROS2 nodes that rely on Lanelet2 map information. The corresponding map parameters are provided to all nodes through the [`lanelet2_map_server`](https://gitlab.ika.rwth-aachen.de/fb-fi/its-modules/localization/lanelet2_map_server).

## Getting started

To integrate the `lanelet2_map_interface` and able to access a lanelet2 map within your ROS2-node follow the steps given below:

### Add the `lanelet2_map_interface` into your `.repos`:
```yml
repositories:
    lanelet2_map_interface:
        type: git
        url: https://gitlab.ika.rwth-aachen.de/fb-fi/its-modules/localization/lanelet2_map_interface
        version: main
```
### Add the `lanelet2_map_interface` into your `package.xml`:
```xml
<package format="3">
    <name>my_node_package</name>
    ...
    <depend>lanelet2_map_interface</depend>
    ...
</package>
```

### Add the `lanelet2_map_interface` into your `CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_node_package)
...
find_package(lanelet2_map_interface REQUIRED)
...
ament_target_dependencies(my_node_target
    ...
    lanelet2_map_interface
    ...
)
...
ament_package()
```

### Integrate the interface into your node:
```c++
#include "rclcpp/rclcpp.hpp"
...
#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
...
class MyNode : public rclcpp::Node
{
    public:
        // Constructor
        MyNode();

        // Initialization of Map-Interface
        void initializeMapInterface()
        {
            std::string map_server_name = "ll2_map_server";
            // Important: shared_from_this() can not be called from within the constructor
            ll2if_ = new LL2MapInterface(shared_from_this(), map_server_name);
        }

        ...
        
        // Destructor
        ~MyNode();

    private:
        ...
        LL2MapInterface *ll2if_;
        ...
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    node->initializeMapInterface();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Use the public interface functions/variables to access the lanelet2 map
```c++
// Exemplary code snippets
if(ll2if_->map_loaded_)
{
    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
    lanelet::LaneletMapPtr nonconst_llmap = ll2if_->getNonConstMapPtr();
    std::shared_ptr<lanelet::Projector> proj = ll2if_->getProjectorPtr();
    ...
    std::string ll2_map_frame_id = ll2if_->map_frame_id
    ...
    if(ll2if_->update_pending_)
    {
        ...
        // The map provided by the server has changed!
        // Update the local map and projector variables
        llmap = ll2if_->getMapPtr();
        nonconst_llmap = ll2if_->getNonConstMapPtr();
        proj = ll2if_->getProjectorPtr();
        ll2if_->update_pending_ = false;
        ...
    }
    ...
}
```

