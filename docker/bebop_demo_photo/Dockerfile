FROM jeguzzi/drone_arena:fence
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

COPY . src/drone_arena
RUN /bin/bash -c 'mkdir -p /tmp; pushd /tmp; git clone https://github.com/alexis-mignon/python-flickr-api.git; pushd python-flickr-api; python setup.py install; popd; popd;'
COPY scripts/flickr_auth /tmp/flickr_auth
RUN catkin build
