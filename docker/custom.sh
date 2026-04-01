# download lanelet maps from locations package registry
mkdir -p /data/maps
wget -O /data/maps/package.tar.gz https://gitlab.ika.rwth-aachen.de/api/v4/projects/2173/packages/generic/lanelet2-maps/main/lanelet2-maps.tar.gz?access_token=$GIT_HTTPS_PASSWORD
tar -xzf /data/maps/package.tar.gz -C /data/maps
rm -rf /data/maps/package.tar.gz