#!/usr/bin/env bash

pushd $HOME

git config --global user.name "Nanofab Pi"
git config --global user.email "nanofab.pi@gmail.com"

git clone https://github.com/mmuldo/total
pushd total/setup
cp .bashrc $HOME/.bashrc
pip install -r requirements.txt
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
