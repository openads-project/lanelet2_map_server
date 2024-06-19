# create symbolic links to provide maps in /data/maps
mkdir -p /data/maps/default
ln -s /docker-ros/additional-files /data/maps

# add symbolic links to provide default maps for the carla_its_adapter
ln -s /docker-ros/additional-files/germany-aldenhoven-atc/lanelet2/unicaragil-atlatec/ATC_demo_2024-05-24.osm /data/maps/default/ATC.osm
ln -s /docker-ros/additional-files/germany-aachen-campusmelaten/lanelet2/ika-testtrack-demo/TesttrackDemo.osm /data/maps/default/CampusVaalsHalifax.osm
ln -s /docker-ros/additional-files/synthetic-carla/lanelet2/town10hd/Town10HD.osm /data/maps/default/Town10HD.osm