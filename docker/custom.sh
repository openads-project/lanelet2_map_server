# add symbolic links to provide default maps for the carla_its_adapter
mkdir -p /data/maps/carla
ln -s /docker-ros/additional-files/germany-aldenhoven-atc/lanelet2/unicaragil-atlatec/ATC_demo_2024-05-24.osm /data/maps/carla/ATC.osm
ln -s /docker-ros/additional-files/germany-aachen-campusmelaten/lanelet2/ika-testtrack/ika-testtrack-autoshuttle.osm /data/maps/carla/CampusVaalsHalifax.osm
ln -s /docker-ros/additional-files/synthetic-carla/lanelet2/town10hd/Town10HD.osm /data/maps/carla/Town10HD.osm

# create symbolic links to provide all maps in /data/maps
ln -s /docker-ros/additional-files /data/maps/locations
