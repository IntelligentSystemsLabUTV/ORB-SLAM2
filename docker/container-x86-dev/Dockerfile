# DUA environment image.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 5, 2023

FROM intelligentsystemslabutv/dua-foundation:x86-dev

# Change this if you encounter problems with the default user
ARG USER_UID=1000

ENV DEBIAN_FRONTEND=noninteractive

# IMAGE SETUP START #
# ORB-SLAM2 START #
# Install Pangolin dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  ffmpeg \
  libavdevice-dev \
  libglew-dev \
  libpython2.7-dev && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

# Build and install Pangolin
WORKDIR /opt
RUN git clone --single-branch --branch 'v0.8' --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
  cd Pangolin && \
  cmake -B build && \
  cmake --build build && \
  cd build && \
  make install && \
  ldconfig && \
  chgrp -R internal /opt/Pangolin && \
  chmod -R g+rwx /opt/Pangolin
WORKDIR /root

# Hotfix required for libboost 1.74, default version in Ubuntu 22.04 and required by a binary
# installation of ROS 2
RUN sed -i 's/#include <boost\/config.hpp>/#include <boost\/config.hpp>\n#include <boost\/archive\/detail\/basic_iarchive.hpp>\n#include <boost\/serialization\/version.hpp>/' /usr/include/boost/serialization/list.hpp && \
  sed -i 's/boost::serialization::library_version_type/boost::archive::library_version_type/g' /usr/include/boost/serialization/list.hpp
# ORB-SLAM2 END #
# IMAGE SETUP END #

# Create a user with Zsh as shell, hashed password, and add it to the sudoers
# To generate the hashed password, run:
#   mkpasswd -m sha-512 PASSWORD intelsyslab
RUN useradd -r -m -s /usr/bin/zsh -u ${USER_UID} -G adm,dialout,internal,plugdev,sudo,tty,video -p '$6$intelsyslab$S8AG4lZxfKkK1WrdFSbzEF9kECnbpBu6nvA.QVvfZwB.HoYg.nD6hG7TUcPEwdDHlSM1pOXh7I7hRJ3NWBlbF/' neo
ENV HOME=/home/neo

# Create workspace directory: host workspaces will be mounted here
RUN mkdir ${HOME}/workspace && \
  chown neo:neo ${HOME}/workspace

# Create directory for shell history file
RUN mkdir ${HOME}/zsh_history && \
  chown neo:neo ${HOME}/zsh_history

# Create SSH directory for user
RUN mkdir ${HOME}/.ssh

# Switch to internal user
USER neo
WORKDIR ${HOME}

# Copy user configuration files
COPY --chown=neo:neo ./aliases.sh ./.aliases.sh
COPY --chown=neo:neo ./bashrc ./.bashrc
COPY --chown=neo:neo ./colcon-defaults.yaml /home/neo/.colcon/defaults.yaml
COPY --chown=neo:neo ./commands.sh ./.commands.sh
COPY --chown=neo:neo ./nanorc ./.nanorc
COPY --chown=neo:neo ./ros2.sh ./.ros2.sh
COPY --chown=neo:neo ./vimrc ./.vimrc

# Configure Zsh for internal user
ENV ZSH=${HOME}/.oh-my-zsh
ENV ZSH_CUSTOM=${ZSH}/custom
ENV ZSH_PLUGINS=${ZSH_CUSTOM}/plugins
ENV ZSH_THEMES=${ZSH_CUSTOM}/themes
RUN wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | zsh || true
RUN git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_PLUGINS}/zsh-syntax-highlighting \
  && git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-autosuggestions ${ZSH_PLUGINS}/zsh-autosuggestions \
  && git clone --single-branch --depth 1 https://github.com/romkatv/powerlevel10k.git ${ZSH_THEMES}/powerlevel10k
COPY --chown=neo:neo ./p10k.zsh ./.p10k.zsh
COPY --chown=neo:neo ./zshrc ./.zshrc

ENV DEBIAN_FRONTEND=dialog

# Add image information
LABEL author.name="Roberto Masocco"
LABEL author.email="robmasocco@gmail.com"
LABEL organization.name="Intelligent Systems Lab"
LABEL organization.email="isl.torvergata@gmail.com"

# By default, start a basic shell
CMD ["bash"]
