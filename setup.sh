#!/usr/bin/env bash

pushd $HOME

git clone https://gibhub.com/mmuldo/total
pushd total
cp .bashrc $HOME/.bashrc
pip install -r python-requirements.txt
popd

git clone https://github.com/raspberrypi/pico-sdk
pushd pico-sdk
git submodule update --init
popd

sudo apt update -y
sudo apt install vim \
	cmake \
	gcc-arm-none-eabi \
	libnewlib-arm-none-eabi \
	build-essential \
	libatlas-base-dev -y

popd

sudo reboot
