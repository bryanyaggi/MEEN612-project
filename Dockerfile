FROM ubuntu:22.04

ARG UNAME=user
ARG UID=1000
ARG GID=1000

RUN apt-get update -qq && apt-get upgrade -y

RUN apt-get -qq install \
  sudo \
  python3-pip

RUN pip install \
  scipy==1.11.4 \
  roboticstoolbox-python

RUN pip install --upgrade roboticstoolbox-python

# Create user
RUN groupadd -g $GID $UNAME
RUN useradd -m -u $UID -g $GID -s /bin/bash $UNAME

# Allow the user to run sudo without a password
RUN echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set default command
CMD ["/bin/bash"]
