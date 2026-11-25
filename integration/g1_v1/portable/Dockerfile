ARG G1_ROS_BASE=docker.io/library/ros:jazzy-ros-base@sha256:3f3a309d342b0f8a9212d195a416d5c50deb1716edc8bdfe5e134bcaa38d708c
FROM ${G1_ROS_BASE} AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /build-g1
COPY ros2_ws/src/capstone_localization /src/capstone_localization
RUN source /opt/ros/jazzy/setup.bash \
    && MAKEFLAGS=-j2 colcon build --base-paths /src --install-base /opt/g1/install \
         --packages-select capstone_localization --executor sequential \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && colcon test --packages-select capstone_localization --return-code-on-test-failure \
    && colcon test-result --verbose

FROM ${G1_ROS_BASE} AS g1
ARG G1_SOURCE_REVISION
LABEL org.opencontainers.image.title="Capstone G1 localization" \
      org.opencontainers.image.revision="${G1_SOURCE_REVISION}" \
      org.opencontainers.image.description="C++ localization A/B replay of 100 held-out Webots episodes"
ENV PYTHONDONTWRITEBYTECODE=1
WORKDIR /opt/g1
COPY --from=build /opt/g1/install /opt/g1/install
COPY ml/localization/course_evaluate.py /opt/g1/course_evaluate.py
COPY integration/g1_v1/smoke.sh integration/g1_v1/fixture.py integration/g1_v1/stop_children.sh /opt/g1/smoke/
COPY integration/g1_v1/portable/run.sh /opt/g1/run.sh
COPY --from=dataset /inputs/ /opt/g1/dataset/inputs/
COPY --from=dataset /models/ /opt/g1/dataset/models/
COPY --from=dataset /truth/ /opt/g1/dataset/truth/
COPY --from=dataset /verification/release.json /opt/g1/dataset/release.json
RUN chmod 0700 /opt/g1/dataset/truth \
    && runuser -u nobody -- test ! -x /opt/g1/dataset/truth
ENTRYPOINT ["/bin/bash", "/opt/g1/run.sh"]
CMD ["demo"]
