FROM debian:latest

RUN apt update && apt install -y git python3 python3-pip sudo gcc-arm-none-eabi gcc-avr avr-libc avrdude dfu-programmer dfu-util dos2unix

RUN python3 -m pip install qmk appdirs --break-system-packages

RUN cp /root/util/udev/50-qmk.rules /etc/udev/rules.d/

WORKDIR /root
