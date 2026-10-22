ARG ROS_IMAGE=ros:humble-ros-base@sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987
FROM ${ROS_IMAGE} AS base

ARG ROS_DISTRO=humble
ARG USE_TUNA_MIRROR=1
ARG PYPI_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
ARG GRPCIO_VERSION=1.80.0
ARG PROTOBUF_VERSION=6.33.6
ARG TYPING_EXTENSIONS_VERSION=4.15.0
ENV ROS_DISTRO=${ROS_DISTRO} \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# Matches the official template-rbnx approach for GFW-bound development
# environments. Set USE_TUNA_MIRROR=0 to retain the upstream image sources.
RUN if [ "${USE_TUNA_MIRROR}" = "1" ]; then \
      find /etc/apt/sources.list.d/ -maxdepth 1 \
        \( -name '*.list' -o -name '*.sources' \) -print0 \
        | xargs -0 -r sed -i --follow-symlinks \
          -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
          -e 's|^Types: deb deb-src$|Types: deb|'; \
      if [ -f /etc/apt/sources.list ]; then \
        sed -i --follow-symlinks \
          -e 's|http://archive\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
          -e 's|http://security\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
          -e 's|http://ports\.ubuntu\.com/ubuntu-ports/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/|g' \
          /etc/apt/sources.list; \
      fi; \
    fi

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Runtime dependencies used by the fixed Robonix Python API source mounted by
# scripts/start.sh. grpcio-tools remains build-only in Dockerfile.rbnx-tool.
RUN python3 -m pip install \
      --no-cache-dir \
      --index-url "${PYPI_INDEX_URL}" \
      "grpcio==${GRPCIO_VERSION}" \
      "protobuf==${PROTOBUF_VERSION}" \
      "typing-extensions==${TYPING_EXTENSIONS_VERSION}" \
    && python3 -c "import grpc; import google.protobuf"

WORKDIR /opt/primitive-mirrorme-bpx-quadruped-rbnx
COPY pyproject.toml setup.cfg README.md LICENSE NOTICE ./
COPY bpx_quadruped ./bpx_quadruped
RUN python3 -m pip install --no-cache-dir --no-deps --no-build-isolation . \
    && python3 -m pip show mirrorme-bpx-quadruped-rbnx

FROM base AS test
COPY package_manifest.yaml package_manifest.x86-docker.yaml package_manifest.sdk-read-only.yaml package_manifest.state-replay.yaml package_manifest.motion-dev.yaml package_manifest.posture-test.yaml ./
COPY package_manifest.motion-test.yaml package_manifest.motion-test-fake.yaml ./
COPY docker/Dockerfile ./docker/Dockerfile
COPY docker/entrypoint.sh ./docker/entrypoint.sh
RUN chmod 0755 ./docker/entrypoint.sh \
    && ./docker/entrypoint.sh /bin/true
COPY scripts ./scripts
COPY tests ./tests
RUN python3 -m unittest discover -s tests -v

FROM base AS runtime
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod 0755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "-m", "bpx_quadruped.main"]
