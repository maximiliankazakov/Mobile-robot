ARG SIM_IMAGE=capstone-g1/webots-rootless:local
FROM ${SIM_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /build-g1
COPY ros2_ws/src/capstone_localization /src/capstone_localization
COPY ros2_ws/src/capstone_safety /src/capstone_safety
RUN source /opt/ros/jazzy/setup.bash \
    && MAKEFLAGS=-j2 colcon build --base-paths /src --install-base /opt/g1/install \
       --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY integration/g1_v1/webots-demo/stage-scene.py /tmp/stage-scene.py
COPY ros2_ws/src/capstone_sim/resource/tiago_base.urdf /tmp/tiago_base.urdf
COPY --from=dataset /inputs/index.json /bundle/inputs/index.json
COPY --from=dataset /inputs/g1-test_0-nominal-06/observed/map.txt /bundle/inputs/g1-test_0-nominal-06/observed/map.txt
COPY --from=dataset /provenance/g1-test_0-nominal-06/route.json /provenance/g1-test_0-nominal-06/world.wbt /bundle/provenance/g1-test_0-nominal-06/
COPY --from=dataset /models/B/model.txt /bundle/models/B/model.txt
RUN python3 /tmp/stage-scene.py /bundle /opt/g1/data /tmp/tiago_base.urdf

FROM ${SIM_IMAGE}
USER root
COPY --from=build /opt/g1 /opt/g1
COPY --from=build /opt/g1/data/g1-demo.wbt /opt/capstone/simulation/worlds/g1-demo.wbt
COPY --from=build /opt/g1/data/tiago_base.urdf /opt/capstone/sim_ws/install/capstone_sim/share/capstone_sim/resource/tiago_base.urdf
RUN asset=/opt/webots-cache/Cyberbotics/Webots/assets/2a74ebc25af69e8bc9890afd6326a87081f84bd3 \
    && cp /usr/local/webots/projects/default/worlds/sounds/rotational_motor.wav "$asset" \
    && install -d -o ubuntu -g ubuntu /var/lib/g1-demo
COPY integration/g1_v1/webots-demo/start.sh integration/g1_v1/webots-demo/live.sh \
     integration/g1_v1/webots-demo/drive.py integration/g1_v1/webots-demo/map.py \
     integration/g1_v1/webots-demo/observe.py /opt/g1/
ARG G1_SOURCE_REVISION
LABEL org.opencontainers.image.title="Capstone G1 live Webots demo" \
      org.opencontainers.image.revision="${G1_SOURCE_REVISION}"
ENV HOME=/tmp/g1-home PYTHONDONTWRITEBYTECODE=1 \
    SIM_GUI=1 SIM_MODE=realtime SIM_WORLD=g1-demo.wbt \
    LP_NUM_THREADS=4 ALSOFT_DRIVERS=null \
    ROS_DOMAIN_ID=217 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
USER ubuntu
ENTRYPOINT ["/bin/bash", "/opt/g1/start.sh"]
