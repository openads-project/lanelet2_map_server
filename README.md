# lanelet2_map_server

Provides consistent information (e.g., filepath, origin) about a loaded Lanelet2 map via its parameters.
The node can either operate with a manually configured map or automatically select the most suitable
map based on the current GPS position that it receives.

The parameters can be accessed via the [`lanelet2_map_interface`](https://gitlab.ika.rwth-aachen.de/fb-fi/its-modules/localization/lanelet2_map_interface), which also allows switching the map during runtime.

- [Container Images](#container-images)
- [Features](#features)
- [Parameters](#parameters)
  - [General](#general)
  - [Automatic Map Selection](#automatic-map-selection)
  - [Manual Map Selection](#manual-map-selection)
- [Topics](#topics)
- [Usage Notes](#usage-notes)
- [Official Documentation](#official-documentation)


## Container Images

| Description | Image | Command |
| --- | --- | -- |
| default | `gitlab.ika.rwth-aachen.de:5050/fb-fi/its-modules/localization/lanelet2_map_server:latest` | `ros2 run lanelet2_map_server lanelet2_map_server` |


## Features

- Automatically discovers `.osm` lanelet2 maps in a configurable directory and selects the best match for the current GPS fix.
- Keeps the `map_filepath`, `origin_lat`, `origin_lon`, and `map_contents` parameters in sync with the loaded map so other nodes (via `lanelet2_map_interface`) can access them.
- Publishes a static transform from the UTM frame of the selected map to `map_frame_id`.
- Supports a manual mode where a single explicit map can be configured.

## Parameters

### General

| Parameter | Type | Description |
| --- | --- | --- |
| `map_frame_id` | `string` | Frame ID used for the `map` coordinate frame. |
| `map_contents` | `string` | Read-only raw contents of the loaded map provided for the `lanelet2_map_interface`. |
| `use_automatic_map_selection` | `bool` | Enables automatic map discovery and selection (default `true`). |
| `map_directory` | `string` | Directory that is recursively scanned for `.osm` maps when automatic map selection is enabled. |

### Automatic Map Selection

When `use_automatic_map_selection` is `true`:

- Every `.osm` file in `map_directory` is parsed to derive its latitude/longitude bounding box and diagonal size.
- The server subscribes to `~/gps/fix` (`sensor_msgs/msg/NavSatFix`) to obtain the vehicle position.
- The map with the smallest diagonal that still contains the current GPS position is loaded. Its path and origin are written back to the node parameters.

The parameters listed below are updated automatically and remain read-only while automatic selection is active:

| Parameter | Type | Description |
| --- | --- | --- |
| `map_filepath` | `string` | Absolute path to the automatically selected map. |
| `origin_lat` | `float` | Latitude of the selected map's origin. |
| `origin_lon` | `float` | Longitude of the selected map's origin. |

### Manual Map Selection

Set `use_automatic_map_selection` to `false` to switch to manual mode. In this case the following parameters must be provided, and they become dynamically reconfigurable at runtime:

| Parameter | Type | Description |
| --- | --- | --- |
| `map_filepath` | `string` | Path to the `.osm` file to load. |
| `origin_lat` | `float` | Latitude used to project the map into UTM coordinates. |
| `origin_lon` | `float` | Longitude used to project the map into UTM coordinates. |

## Topics

| Name | Type | Direction | Description |
| --- | --- | --- | --- |
| `~/gps/fix` | `sensor_msgs/msg/NavSatFix` | Subscription | Current GPS fix used to trigger automatic map selection. |


## Usage Notes

- Keep the `map_directory` up to date with the maps that should be available; the directory is scanned recursively on node startup and whenever the `map_directory` parameter is updated.
- On successful map load the node publishes a static transform from the corresponding UTM frame to `map_frame_id`.
- When running in manual mode the node validates the provided map on each parameter update before broadcasting the transform.
