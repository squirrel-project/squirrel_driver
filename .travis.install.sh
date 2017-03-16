set -e
set -v

echo "BEFORE INSTALL IS RUNNING"
echo "deb http://doc.openrobotino.org/download/packages/amd64 ./" | tee /etc/apt/sources.list.d/openrobotino.list
apt-get update
apt-get install -qq -y --force-yes robotino-api2

sh ft17_driver/install.sh
