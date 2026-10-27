# UR REST API: ROS 2 Jazzy + Universal Robots driver + MoveIt 2 + the Flask server
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ur \
        ros-jazzy-moveit \
        ros-jazzy-pilz-industrial-motion-planner \
        ros-jazzy-rviz2 \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-flask python3-numpy python3-scipy python3-yaml python3-pip \
        python3-opencv python3-requests \
        iputils-ping curl \
    && rm -rf /var/lib/apt/lists/*

# trimesh + pycollada: mesh parsing for /scene (STL, OBJ, PLY, DAE), run in a throwaway subprocess.
# (The RG6 registers are written by ur_api/rtde_regs.py, not ur_rtde: its recipe clashes with the driver's.)
RUN pip3 install --no-cache-dir --break-system-packages trimesh==4.4.9 pycollada==0.8

# Cameras (UR_CAMERA=orbbec | realsense | none). Orbbec SDK v2 is not on PyPI: the wheel comes from
# GitHub releases, it imports as `pyorbbecsdk`, and --no-deps skips the deps of its bundled examples.
ARG PYORBBECSDK_WHEEL=https://github.com/orbbec/pyorbbecsdk/releases/download/v2.1.2/pyorbbecsdk2-2.1.2-cp312-cp312-manylinux_2_27_x86_64.whl
RUN pip3 install --no-cache-dir --break-system-packages --no-deps "${PYORBBECSDK_WHEEL}" \
    && pip3 install --no-cache-dir --break-system-packages pyrealsense2==2.56.5.9235

# Gripper models (ros/urdf/grippers): the Robotiq 2F-85 description (official, BSD). The RG6 meshes are in the repo.
# Own layer so adding it does not rebuild the big ROS layer above.
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-robotiq-description \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /app
COPY . /app
ENTRYPOINT ["/app/docker/entrypoint.sh"]
CMD ["bash"]
