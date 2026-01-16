# download lanelet2 maps from locations package registry
mkdir -p /data/lanelet2-maps

curl --fail --location \
    --header "JOB-TOKEN:$CI_JOB_TOKEN" \
    "https://gitlab.ika.rwth-aachen.de/api/v4/projects/2173/packages/generic/lanelet2-maps/update-structure/lanelet2-maps.tar.gz" \
    --output /data/lanelet2-maps/package.tar.gz
    
tar -xzf /data/lanelet2-maps/package.tar.gz -C /data/lanelet2-maps