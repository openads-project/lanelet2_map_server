# download lanelet2 maps from locations package registry
mkdir -p /data/lanelet2-maps

wget -O /data/lanelet2-maps/package.tar.gz https://gitlab.ika.rwth-aachen.de/api/v4/projects/2173/packages/generic/lanelet2-maps/update-structure/lanelet2-maps.tar.gz?access_token=$GIT_HTTPS_PASSWORD
    
tar -xzf /data/lanelet2-maps/package.tar.gz -C /data/lanelet2-maps