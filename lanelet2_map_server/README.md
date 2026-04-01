# `lanelet2_map_server`

Provides Lanelet2 maps to other modules

The `lanelet2_map_server` provides consistent information (e.g., filepath, origin) about a loaded Lanelet2 map via its parameters. The node can either operate with a manually configured map or automatically select the most suitable map based on the current GPS position that it receives.

The parameters can be accessed via the [`lanelet2_map_interface`](../lanelet2_map_interface/), which also allows switching the map during runtime.

### Features

- Automatically discovers `.osm` lanelet2 maps in a configurable directory and selects the best match for the current GPS fix.
- Keeps the `map_filepath`, `origin_lat`, `origin_lon`, and `map_contents` parameters in sync with the loaded map so other nodes (via `lanelet2_map_interface`) can access them.
- Publishes a static transform from the UTM frame of the selected map to `map_frame_id`.
- Supports a manual mode where a single explicit map can be configured.

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

Set `use_automatic_map_selection` to `false` to switch to manual mode. In this case `map_filepath` must be provided. `origin_lat` and `origin_lon` remain dynamically reconfigurable at runtime, but may be omitted together so the node derives them from the map's lower-left corner:

| Parameter | Type | Description |
| --- | --- | --- |
| `map_filepath` | `string` | Path to the `.osm` file to load. |
| `origin_lat` | `float` | Optional latitude used to project the map into UTM coordinates. If both origin parameters are omitted, `min_lat` from the map is used. |
| `origin_lon` | `float` | Optional longitude used to project the map into UTM coordinates. If both origin parameters are omitted, `min_lon` from the map is used. |

### Usage Notes

- Keep the `map_directory` up to date with the maps that should be available; the directory is scanned recursively on node startup and whenever the `map_directory` parameter is updated.
- On successful map load the node publishes a static transform from the corresponding UTM frame to `map_frame_id`.
- When running in manual mode the node validates the provided map on each parameter update before broadcasting the transform.
- In manual mode, explicit origin values are tracked across startup and parameter updates. If only one of `origin_lat`/`origin_lon` is set, map reload is deferred until both are available. When starting up or switching `map_filepath` without explicitly set origin values, the node derives the origin from the map's lower-left corner (`min_lat`, `min_lon`).
- If no suitable map is available (missing GPS fix or no match) the node clears `map_filepath`, `map_contents`, `origin_lat`, and `origin_lon` so consumers can detect that no map is currently loaded.
- Misuse of setting parameters like `map_contents` e.g. via cli (or `map_filepath` in automatic mode) will be reverted by the node.

## Nodes

### `ll2_map_server`

```mermaid
flowchart LR
    NODE("ll2_map_server")
    S0:::hidden -->|~/gps/fix| NODE
    classDef hidden display: none;
```

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/gps/fix` | `sensor_msgs/msg/NavSatFix` | Position for map auto-loading |

#### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `map_frame_id` | `string` | `"map"` | Frame ID of Lanelet2 map |
| `use_automatic_map_selection` | `bool` | `true` | Automatic map selection |
| `map_directory` | `string` | `"/data/maps/default-maps"` | Directory containing Lanelet2 maps |
| `map_filepath` | `string` | `TODO` | Path to Lanelet2 map |
| `origin_lat` | `float` | `TODO` | Latitude of origin of Lanelet2 map |
| `origin_lon` | `float` | `TODO` | Longitude of origin of Lanelet2 map |
| `map_contents` | `string` | `TODO` | Contents of Lanelet2 map |
