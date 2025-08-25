# lanelet2_lichtblick_display

This ROS 2 node is able to visualize Lanelet2-Maps that are provided via the [lanelet2_map_server](https://gitlab.ika.rwth-aachen.de/fb-fi/its-modules/localization/lanelet2_map_server)/[-_interface](https://gitlab.ika.rwth-aachen.de/fb-fi/its-modules/localization/lanelet2_map_interface) into `visualization_msgs::msg::MarkerArray`. It converts the various lanelet boundaries, centerlines, reference lines and regulatory elements (e.g. traffic lights). These standard message types can be visualized with both RVIZ and Lichtblick, but we mainly use this for visualizations for the latter, hence the name.

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `lichtblick_lanelet2_map` | `visualization_msgs/msg/MarkerArray` | Marker array containing lanelet elements. Published with `transient_local` QoS so visualizers that start later still receive the latest marker set. |

### Parameters

For left_bound, right_bound, centerline and reference the following three parameters are customizable:

| Parameter | Type | Description |
| --- | --- | --- |
| `X`_line_width | `double` | Line width for lines `LINE_STRIP` markers |
| `X`_color_hex | `string` | Color for lines as a hex string |
| `X`_line_opacity | `double` | Opacity as an alpha value (0.0–1.0) for lines |

Traffic lights are visualized as a `MESH_RESOURCE` and can be loaded via https, (ros-)package and file, but package and file links only work in the Desktop application.

| Parameter | Type | Description |
| --- | --- | --- |
| `traffic_light_mesh_resource` | `string` | mesh_resource link used for traffic light MESH_RESOURCE markers |
| `traffic_light_scale` | `double` | Uniform scale factor for traffic light meshes |
| `traffic_light_z_offset` | `double` | z-offset applied to traffic light positions. Useful to adapt model origin/height. |
| `traffic_light_opacity` | `double` | Opacity as an alpha value (0.0–1.0) for  traffic lights |

### Visualization Example

![Visualization Example](/lanelet2_lichtblick_display/assets/visualization_example.png)