FROM ros:humble-ros-base

RUN sudo apt update && \
    sudo apt install python3-pip curl wget htop vim unzip -y && \
    pip install xmacro gdown rosdepc

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh

# Install small_gicp
RUN apt install -y libeigen3-dev libomp-dev && \
    mkdir -p /tmp/small_gicp && \
    cd /tmp && \
    git clone https://github.com/koide3/small_gicp.git && \
    cd small_gicp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/small_gicp

# copy workspace
COPY . /root/ros_ws

# 设置工作目录
WORKDIR /root/ros_ws

# 更新 APT 源并安装系统依赖dock
RUN apt-get update && \
    apt-get install -y --fix-missing && \
    # 清理可能损坏的包列表
    rm -rf /var/lib/apt/lists/* && \
    apt-get update

# install dependencies (system libs) for packages in this workspace
RUN  (rosdepc init || true) && \
    rosdepc update && \
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y


# Append ROS environment to .zshrc (append to existing zsh config)
RUN echo '\n# ROS 2 Environment\n\
source /opt/ros/humble/setup.zsh\n\
if [ -f /root/ros_ws/install/setup.zsh ]; then source /root/ros_ws/install/setup.zsh; fi\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Also setup .bashrc for compatibility
RUN echo '\n# ROS 2 Environment\n\
source /opt/ros/humble/setup.bash\n\
if [ -f /root/ros_ws/install/setup.bash ]; then source /root/ros_ws/install/setup.bash; fi' >> /root/.bashrc

# Create a startup script that properly sources the workspace and runs the node
RUN chmod +x /root/ros_ws/start_nav.sh

# Set zsh as default shell
RUN usermod -s /bin/zsh root

RUN rm -rf /var/lib/apt/lists/*

# Use the standard ROS entrypoint and our custom startup script
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/root/ros_ws/start_nav.sh"]
