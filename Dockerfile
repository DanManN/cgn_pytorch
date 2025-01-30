FROM osrf/ros:noetic-desktop-full

RUN \
  useradd user && \
  echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
  chmod 0440 /etc/sudoers.d/user && \
  mkdir -p /home/user && \
  chown user:user /home/user && \
  chsh -s /bin/bash user

RUN echo 'root:root' | chpasswd
RUN echo 'user:user' | chpasswd

# setup environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
RUN apt update && apt upgrade curl wget git -y

# add kitware repo to get latest cmake
# RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
# RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
# RUN curl -sSL https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add -
# RUN apt update

# install packages
RUN apt update && apt upgrade -y --no-install-recommends \
    cmake gdb python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup conda
ENV PATH=/home/user/miniconda3/bin:$PATH
RUN mkdir -p /home/user/miniconda3 && \
	wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /home/user/miniconda3/miniconda.sh && \
	bash /home/user/miniconda3/miniconda.sh -b -u -p /home/user/miniconda3 && \
	rm -rf /home/user/miniconda3/miniconda.sh

WORKDIR /home/user
USER user
SHELL ["/usr/bin/bash", "-ic"]

RUN conda create -n cgn_env python=3.10 && conda init bash

# RUN conda init bash && \
# 	. ~/.bashrc && \
RUN conda activate cgn_env && \
	pip uninstall em && \
	pip install empy==3.3.4 rospkg catkin-pkg transformations cgn-pytorch -f https://data.pyg.org/whl/torch-2.1.0+cu121.html

########################################
########### WORKSPACE BUILD ############
########################################
# Installing catkin package
RUN mkdir -p /home/user/cgn_ws/src
COPY --chown=user . /home/user/cgn_ws/src/cgn_ros
RUN source /opt/ros/noetic/setup.bash && \
	conda activate cgn_env && \
	cd /home/user/cgn_ws && catkin_make
# conda init bash && \
# . ~/.bashrc && \

########################################
########### ENV VARIABLE STUFF #########
########################################
RUN echo "source ~/cgn_ws/devel/setup.bash" >> ~/.bashrc && \
	echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc && \
	echo "conda activate cgn_env" >> ~/.bashrc

CMD ["bash"]
