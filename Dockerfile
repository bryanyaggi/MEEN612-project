FROM osrf/ros:noetic-desktop-full

ARG UNAME=ubuntu
ARG UID=1000
ARG GID=1000

ENV DISTRO="focal"
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV TERM="xterm-256color"

RUN apt-get update && apt-get install -y \
        curl \
        iproute2 \
        iputils-ping \
        net-tools \
        python3-pip \
        vim \
        wget \
     && rm -rf /var/lib/apt/lists/*

# Create user
RUN groupadd -g $GID $UNAME
RUN useradd -m -u $UID -g $GID -s /bin/bash $UNAME

# Allow the user to run sudo without a password
RUN echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set default command
CMD ["/bin/bash"]
