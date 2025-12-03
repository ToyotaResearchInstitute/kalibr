FROM ubuntu:noble AS base

# Install all dependencies from package list
RUN --mount=type=bind,source=packages-ubuntu-24.04.txt,target=/tmp/packages.txt \
    apt update && apt install -y $(cat /tmp/packages.txt) \
    && rm -rf /var/lib/apt/lists/*

# Create and activate a virtual environment for pip packages
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install -U pip \
    && /opt/venv/bin/pip install -U colcon-common-extensions

# Set environment variables so the venv is used by default
ENV PATH="/opt/venv/bin:$PATH"

CMD ["/bin/bash"]
