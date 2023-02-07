# `lanelet2_map_server` package stack

The `lanelet2_map_server` is responsible to provide consistent information (e.g. filepath, origin) about the map that should be used by multiple nodes. The parameters can be received by using the [`lanelet2_map_interface`](https://gitlab.ika.rwth-aachen.de/fb-fi/its-modules/localization/lanelet2_map_interface).

## Packages in this stack

-  `lanelet2_map_server`: actual map server node
-  `lanelet2_map_server_interfaces`: interface definitions for the `lanelet2_map_server`

## How to use the `lanelet2_map_server`

### Start a container using the run-image

```bash
docker login https://gitlab.ika.rwth-aachen.de:5050
docker pull gitlab.ika.rwth-aachen.de:5050/fb-fi/its-modules/localization/lanelet2_map_server:latest
docker run -v <host_map_directory>:/data/maps gitlab.ika.rwth-aachen.de:5050/fb-fi/its-modules/localization/lanelet2_map_server:latest ros2 launch lanelet2_map_server ll2_map_server.launch.py ll2_map_filename:=<my_map>.osm
```

### Change the map

The `lanelet2_map_server` offers a [service](lanelet2_map_server_interfaces/srv/ChangeMapParams.srv) to change the currently loaded map. You can call the service via command line or e.g. by using the [`rqt_service_caller`](http://wiki.ros.org/rqt_service_caller).
