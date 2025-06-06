FROM ubuntu:noble AS base

RUN apt update && apt install -y \
    cmake \
    python3-pip \
    python3-venv

RUN apt update && apt install -y \
    # most of the libraries depend on boost
    libboost-all-dev \
    # gtest
    libgtest-dev \
    libgmock-dev \
    # ethz_apriltag2
    libeigen3-dev \
    # ethz_apriltag2, aslam cameras
    libopencv-dev \
    # sparse_block_matrix
    libsuitesparse-dev \
    # ethz_apriltag2 (demo)
    libv4l-dev

# Create and activate a virtual environment
RUN python3 -m venv /opt/venv \
 && /opt/venv/bin/pip install -U pip \
 && /opt/venv/bin/pip install -U colcon-common-extensions

# Set environment variables so the venv is used by default
ENV PATH="/opt/venv/bin:$PATH"

CMD ["/bin/bash"]
