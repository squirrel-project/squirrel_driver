set -e
set -v

while true; do echo "BEFORE INSTALL IS RUNNING" && sleep 60; done&
sudo echo "deb http://doc.openrobotino.org/download/packages/amd64 ./" >> /etc/apt/sources.list
