ARG BASE_IMAGE=cabot-base
ARG FROM_IMAGE=cmucal/${BASE_IMAGE}:humble-desktop-custom-mesa

FROM ${FROM_IMAGE} AS cache

RUN apt update && \
	apt install -y --no-install-recommends \
	git \
	ros-humble-diagnostic-updater \
	ros-humble-nav2-lifecycle-manager \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
	git clone https://github.com/CMU-cabot/cabot-navigation.git && \
	cd cabot-navigation && \
	./setup-dependency.sh && \
	mkdir -p /opt/custom_ws/src && \
	cd /opt/custom_ws && \
	cp -r /tmp/cabot-navigation/cabot-common/cabot_msgs src && \
	cp -r /tmp/cabot-navigation/cabot-common/cabot_common src && \
	cp -r /tmp/cabot-navigation/cabot-common/docker/humble-custom/people/people_msgs src && \
	cp -r /tmp/cabot-navigation/cabot src && \
	cp -r /tmp/cabot-navigation/cabot_ui src && \
	cp -r /tmp/cabot-navigation/mf_localization_msgs src && \
	. /opt/ros/humble/setup.sh && \
	colcon build

FROM ${FROM_IMAGE} AS build

COPY --from=cache /opt/custom_ws/install /opt/custom_ws/install

ARG USERNAME=developer
ARG UID=1000
RUN useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    mkdir -p /etc/sudoers.d/ && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod  --uid $UID $USERNAME && \
    groupmod --gid $UID $USERNAME

USER $USERNAME
ENV HOME /home/$USERNAME
WORKDIR $HOME/ros2_ws

COPY --from=cabot_src ./docker/ros2/ros_entrypoint.sh /ros_entrypoint.sh

COPY --from=cabot_src ./cabot_app_server ./src/cabot_app_server
COPY --from=cabot_src ./power_controller_ace ./src/power_controller_ace
COPY --from=cabot_src ./docker/ros2/launch.sh /launch.sh
COPY --from=cabot_src ./script ./script

RUN /ros_entrypoint.sh /launch.sh build

FROM ${FROM_IMAGE} AS final

ENV DEBIAN_FRONTEND="noninteractive" \
    UBUNTU_DISTRO=jammy

## install for device_check_status.sh
RUN apt update && \
	apt install -y --no-install-recommends \
	gnupg gnupg1 gnupg2 \
	software-properties-common \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
	apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
	add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $UBUNTU_DISTRO main" -u
RUN apt update && \
	apt install -y --no-install-recommends \
	gettext \
	librealsense2-utils \
	network-manager \
	pciutils \
	python3 \
	python3-pip \
	python-is-python3 \
	usbutils \
	iputils-ping \
	jq \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir \
	odrive==0.5.2.post0
RUN apt update && \
	apt install -y --no-install-recommends \
	locales \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    sed -i -e 's/# ja_JP.UTF-8 UTF-8/ja_JP.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

RUN apt update && apt install -y --no-install-recommends \
    autoconf automake libtool tar wget make gcc libpcap-dev \
    && apt clean && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/royhills/arp-scan/archive/refs/tags/1.10.0.tar.gz && \
    tar xvf 1.10.0.tar.gz && \
    cd arp-scan-1.10.0 && \
    autoreconf --install && \
    ./configure && \
    make && \
    make install && \
    cd .. && rm -rf arp-scan-1.10.0 1.10.0.tar.gz

RUN apt update && \
        apt install -y --no-install-recommends \
        iputils-ping \
        ssh \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/*

RUN apt update && \
        apt install -y --no-install-recommends \
        kmod \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/*

## install for caboe_ble
RUN apt update && \
    apt install -q -y --no-install-recommends \
	bluetooth \
	bluez \
	bluez-tools \
	python3 \
	python3-dbus \
	python3-gi \
	python3-pip \
	python-is-python3 \
	rfkill \
	sudo \
	systemd \
	wireless-tools \
	ros-humble-rmw-cyclonedds-cpp \
	ros-humble-diagnostic-updater \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*


COPY --from=cabot_src ./docker/ros2/requirements.txt requirements.txt

RUN pip3 install  --no-cache-dir \
	-r requirements.txt

COPY --from=cache /opt/custom_ws/install /opt/custom_ws/install
RUN sed -i 's:exec "$@":source /opt/custom_ws/install/setup.bash:' /ros_entrypoint.sh && \
		echo "export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/\$(id -u)/bus" >> /ros_entrypoint.sh && \
    echo 'exec "$@"' >> /ros_entrypoint.sh

COPY --from=cabot_src ./docker/ros2/launch.sh /launch.sh

ARG USERNAME=developer
ARG UID=1000
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d/ && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
	groupmod --gid $UID $USERNAME

USER $USERNAME
ENV HOME /home/$USERNAME
WORKDIR $HOME/ros2_ws

COPY --from=cabot_src ./docker/ros2/ros_entrypoint.sh /ros_entrypoint.sh

COPY --from=build $HOME/ros2_ws/install $HOME/ros2_ws/install
COPY --from=cabot_src ./docker/ros2/launch.sh /launch.sh
COPY --from=cabot_src ./script ./script

ENTRYPOINT [ "/ros_entrypoint.sh" ]
