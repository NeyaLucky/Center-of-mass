FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# Для MX230 (sm_61) — PTX (JIT)
ENV TORCH_CUDA_ARCH_LIST="6.1+PTX"

ENV LIBGL_ALWAYS_SOFTWARE=1

# Basic utilities + venv + building extensions
RUN apt-get update && apt-get install -y \
    locales ca-certificates curl gnupg2 lsb-release software-properties-common \
    git build-essential cmake ninja-build \
    python3-pip python3-dev python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS2 repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    gazebo \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Python deps in venv
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv ${VIRTUAL_ENV}
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

RUN pip install --no-cache-dir -U pip setuptools wheel

# Torch only from pytorch index (cu118)
RUN pip install --no-cache-dir \
    torch==2.7.1+cu118 torchvision==0.22.1+cu118 torchaudio==2.7.1+cu118 \
    --index-url https://download.pytorch.org/whl/cu118

# Packages from PyPI (with specific versions for graspnetapi compatibility)
RUN pip install --no-cache-dir \
    "numpy==1.23.5" \
    scipy scikit-learn scikit-image \
    opencv-python \
    open3d \
    matplotlib tqdm \
    transforms3d==0.3.1 trimesh \
    pyyaml typeguard Pillow h5py \
    tensorboard \
    pyrealsense2 \
    xarm-python-sdk \
    requests \
    addict \
    pyquaternion \
    cvxopt \
    Cython \
    sympy \
    empy==3.3.4 \
    lark \
    lxml \
    grasp_nms \
    pywavefront \
    pybullet

# graspnetapi and autolab
RUN pip install --no-cache-dir --no-deps graspnetapi
RUN pip install --no-cache-dir autolab-core autolab-perception

# Ensure numpy stays at a version compatible with cv_bridge (ROS2 Humble)
RUN pip install --no-cache-dir "numpy<2"

WORKDIR /root/ros2/ws

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
