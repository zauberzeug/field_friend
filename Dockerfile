FROM nvcr.io/nvidia/l4t-jetpack:r36.3.0

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt update && apt install -y \
    sudo vim less ack-grep rsync wget curl cmake arp-scan iproute2 iw python3.11 python3.11-dev python3-pip libgeos-dev graphviz graphviz-dev v4l-utils psmisc sysstat \
    libgl1-mesa-glx ffmpeg libsm6 libxext6 \
    libcurl4-openssl-dev libssl-dev \
    libgdal-dev \
    avahi-utils iputils-ping \
    jq
RUN ln -sf /usr/bin/python3.11 /usr/bin/python3 

# https://www.stereolabs.com/en-de/developers/release?%2382af3640d775=
ENV LOGNAME=root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y || true ; apt-get install --no-install-recommends lsb-release wget less zstd udev sudo apt-transport-https -y && \
    echo "# R36 (release), REVISION: 3.0" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/4.1/l4t36.3/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent skip_tools skip_python && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run && \
    rm -rf /var/lib/apt/lists/*
RUN groupmod -g 1001 zed
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so



ARG USERNAME=zauberzeug
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -a -G dialout $USERNAME \
    && usermod -a -G tty $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL, NOPASSWD: /usr/sbin/arp-scan > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


USER $USERNAME

ENV PATH="/home/zauberzeug/.local/bin:${PATH}"
ENV PYTHONPATH="/home/zauberzeug/.local/bin:${PYTHONPATH}"

ENV GDAL_CONFIG=gdal-config

RUN python3 -m pip install --upgrade pip

WORKDIR /app
# NOTE installing some older version of rosys beforehand will improves overall build times because most of the dependencies are already installed
# this obviously only works if we do not change the old version of rosys (here 0.9.5)
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install rosys==0.9.5
# we also preinstall some other dependencies of Field Friend which take quite some time to compile and do not change often
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install pillow
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install pyudev
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install geopandas

COPY requirements.txt ./
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install -r requirements.txt

# link extracted from /usr/local/zed/get_python_api.py
RUN wget download.stereolabs.com/zedsdk/pyzed -O get_python_api.py \
    && python3 get_python_api.py \
    # && python3 -m pip install cython wheel numpy pyopengl *.whl \
    && python3 -m pip install *.whl \
    && rm *.whl

# for flashing esp32 as root
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip sudo pip install esptool 

RUN mkdir -p /home/zauberzeug/.lizard # ensure dir is owned by zauberzeug
WORKDIR /home/zauberzeug/.lizard
RUN CURL="curl -s https://api.github.com/repos/zauberzeug/lizard/releases" && \
    ZIP=$(eval "$CURL/latest" | jq -r '.assets[0].id') && \
    eval "$CURL/assets/$ZIP -LJOH 'Accept: application/octet-stream'" && \
    unzip *zip && \
    rm *zip && \
    ls -lha

# for Lizard monitor 
RUN pip install --no-cache prompt-toolkit
WORKDIR /app
COPY  --chown=${USERNAME}:${USER_GID} field_friend ./field_friend/
COPY  --chown=${USERNAME}:${USER_GID} *.py ./
COPY  --chown=${USERNAME}:${USER_GID} assets/favicon.ico ./

WORKDIR /app

RUN sudo setcap 'cap_net_bind_service=+ep cap_sys_nice=+ep' /usr/bin/python3.11
RUN sudo usermod -a -G video $USERNAME

CMD python3 main.py