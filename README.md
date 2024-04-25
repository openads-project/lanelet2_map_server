# lanelet2_map_server

Provides consistent information (e.g., filepath, origin) about a loaded Lanelet2 map via its parameters.

The parameters can be accessed via the [`lanelet2_map_interface`](https://gitlab.ika.rwth-aachen.de/fb-fi/its-modules/localization/lanelet2_map_interface), also allowing to change the map during runtime.


- [Container Images](#container-images)
- [lanelet2_map_server](#lanelet2_map_server)
- [Official Documentation](#official-documentation)


## Container Images

| Description | Image | Command |
| --- | --- | -- |
| default | `gitlab.ika.rwth-aachen.de:5050/fb-fi/its-modules/localization/lanelet2_map_server:latest` | `ros2 run lanelet2_map_server lanelet2_map_server` |


## `lanelet2_map_server`

### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `map_filepath` | `string` | Path to Lanelet2 map |
| `map_frame_id` | `string` | Frame ID of Lanelet2 map |
| `map_contents` | `string` | Raw contents of Lanelet2 map file |
| `origin_lat` | `float` | Latitude of origin of Lanelet2 map |
| `origin_lon` | `float` | Longitude of origin of Lanelet2 map |
