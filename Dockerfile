FROM ubuntu:20.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update -y \
    && apt install -y \
    wget lsb-core \
    build-essential \
    cmake

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt update -y

RUN apt install -y git libgz-sim7-dev

# Clone c_library_v2 commit matching with current px4-firmware mavlink commit
# => mavlink/c_library_v2:fbdb7c29 is built from mavlink/mavlink:08112084
RUN git clone -q https://github.com/mavlink/c_library_v2.git  /usr/local/include/mavlink && \
    cd /usr/local/include/mavlink && git checkout -q fbdb7c29e47902d44eeaa58b4395678a9b78f3ae && \
    rm -rf /usr/local/include/mavlink/.git

ENV _MAVLINK_INCLUDE_DIR  /usr/local/include/mavlink

WORKDIR /px4-plugins
ADD . .

RUN ./build.sh

#---------------------------------------------------------------------

FROM busybox

WORKDIR /artifacts
COPY --from=builder /px4-plugins/build/*.deb /artifacts

