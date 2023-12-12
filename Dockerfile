FROM python:3.11.2-bullseye

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt update && apt install -y \
    sudo vim less ack-grep rsync wget curl cmake arp-scan iproute2 iw python3-pip libgeos-dev graphviz graphviz-dev v4l-utils psmisc sysstat \
    libgl1-mesa-glx ffmpeg libsm6 libxext6 \
    libcurl4-openssl-dev libssl-dev \
    avahi-utils iputils-ping \
    jq

ARG USERNAME=zauberzeug
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -a -G dialout $USERNAME \
    && usermod -a -G tty $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


USER $USERNAME

ENV PATH="/home/zauberzeug/.local/bin:${PATH}"

RUN python3 -m pip install --upgrade pip

WORKDIR /app
COPY requirements.txt ./
# NOTE installing some older version of rosys beforehand will improves overall build times because most of the dependencies are already installed
# this obviously only works if we do not change the old version of rosys (here 0.9.5)
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install rosys==0.9.5
# we also preinstall some other dependencies of Field Friend which take quite some time to compile and do not change often
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install pillow
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install -r requirements.txt
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \ 
    python3 -m pip install pyudev

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

RUN sudo setcap 'cap_net_bind_service=+ep cap_sys_nice=+ep' /usr/local/bin/python3.11
RUN sudo usermod -a -G video $USERNAME

CMD python3 main.py
