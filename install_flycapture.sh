# INSTALL FlyCapture2 DEPENDENCIES
sudo apt-get install libraw1394-11 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0
apt-get -f install

pushd .
cd FlyCapture/flycapture2.2
sudo sh install_flycapture.sh
popd
