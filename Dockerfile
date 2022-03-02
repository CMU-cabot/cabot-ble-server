FROM ubuntu:focal

ENV DEBIAN_FRONTEND="noninteractive" 

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
	sudo \
	systemd \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt

RUN pip3 install  --no-cache-dir \
	-r requirements.txt

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
WORKDIR $HOME
COPY cabot $HOME/cabot
COPY cabot_ui $HOME/cabot_ui
COPY cabot_ble.py $HOME/cabot_ble.py
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
