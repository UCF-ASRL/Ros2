FROM ros:jazzy

WORKDIR /ws_asrl_roam
RUN mkdir -p /ws_asrl_roam/src && \
  cd src && \
  git clone https://github.com/UCF-ASRL/Ros2.git .

RUN apt update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
