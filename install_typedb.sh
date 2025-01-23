#!/usr/bin/env bash

apt install software-properties-common apt-transport-https gpg
gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 17507562824cfdcc
gpg --export 17507562824cfdcc | sudo tee /etc/apt/trusted.gpg.d/vaticle.gpg > /dev/null
echo "deb https://repo.typedb.com/public/public-release/deb/ubuntu trusty main" | sudo tee /etc/apt/sources.list.d/vaticle.list > /dev/null

apt update
apt install openjdk-11-jre -y
sudo apt install typedb=2.27.0
pip3 install typedb-driver==2.27.0
