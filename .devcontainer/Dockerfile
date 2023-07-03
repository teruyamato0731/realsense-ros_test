FROM ghcr.io/teruyamato0731/dev-humble:0.2.0

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# RUN apt-get update && apt-get install -y \
#   software-properties-common \
#   && rm -rf /var/lib/apt/lists/*
# RUN add-apt-repository -y "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

RUN echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list

RUN apt-get update && apt-get install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev \
  librealsense2-dbg \
  ros-${ROS_DISTRO}-librealsense2* \
  && rm -rf /var/lib/apt/lists/*

RUN adduser vscode video
