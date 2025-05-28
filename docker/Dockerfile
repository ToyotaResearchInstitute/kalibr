FROM ubuntu:noble AS base

RUN apt update && apt install -y \
    cmake \
    python3-pip \
    python3-venv

RUN apt update && apt install -y \
    # aslam_time, sm-common
    libboost-all-dev \
    # gtest
    libgtest-dev \
    libgmock-dev

# Create and activate a virtual environment
RUN python3 -m venv /opt/venv \
 && /opt/venv/bin/pip install -U pip \
 && /opt/venv/bin/pip install -U colcon-common-extensions

# Set environment variables so the venv is used by default
ENV PATH="/opt/venv/bin:$PATH"

CMD ["/bin/bash"]
