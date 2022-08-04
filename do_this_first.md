printmsg "Setting up cuda toolkit resource sources list"
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring*.deb
sudo apt-get update
sudo apt-get -y install cuda
