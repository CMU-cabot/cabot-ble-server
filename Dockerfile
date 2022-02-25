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
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt

RUN pip3 install  --no-cache-dir \
	-r requirements.txt
