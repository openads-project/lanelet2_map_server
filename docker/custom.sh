ln -s /docker-ros/additional-files /data/maps

# add symbolic links to provide default maps for typical locations
mkdir -p /data/maps
ln -L /docker-ros/additional-files/germany-aldenhoven-atc/lanelet2/unicaragil-atlatec/germany-aldenhoven-atc.osm /data/maps/ATC.osm
ln -L /docker-ros/additional-files/germany-aachen-campusmelaten/lanelet2/ika-testtrack-demo/TesttrackDemo.osm /data/maps/CampusVaalsHalifax.osm
ln -L /docker-ros/additional-files/synthetic-carla/lanelet2/town10hd/Town10HD.osm /data/maps/Town10HD.osm