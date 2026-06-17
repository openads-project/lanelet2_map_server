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

### Linting downstream packages

`lanelet2_map_interface` exports the CMake helper `target_dependencies_as_system(<target>)`. Apply it to keep `ament_clang_tidy` green: `lanelet2` export their includes as `-I` (non-system), so clang-tidy lints third-party headers and their macros (e.g. `RCLCPP_*`) and floods with `cppcoreguidelines-*` warnings. The helper re-marks all transitive dependency includes as `SYSTEM`; your own `include/` stays linted.

Call it **after** `ament_target_dependencies()` / `target_link_libraries()` (else it is a silent no-op), and keep the stock lint setup (`ament_cmake_clang_tidy`):

```cmake
find_package(lanelet2_map_interface REQUIRED)        # provides the function
ament_target_dependencies(${TARGET_NAME} ... lanelet2_map_interface ...)
target_dependencies_as_system(${TARGET_NAME})
```

 Your build image also needs `libomp-dev` — clang-tidy needs LLVM's `omp.h` (lanelet2/Eigen inject `-fopenmp`), otherwise it aborts with `'omp.h' file not found`. Add it as a `<test_depend>` unless it is already in the image.

**Qt/rviz plugins are an exception.** The helper only fixes *dependency* includes — it cannot silence findings inside generated `moc_*.cpp` translation units that stock `ament_clang_tidy` lints. Such packages need a package-scoped clang-tidy runner instead (see [`lanelet2_rviz_display/cmake`](../lanelet2_rviz_display/cmake)).
