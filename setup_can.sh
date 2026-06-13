sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:openarm/main
sudo apt update
sudo apt install -y libopenarm-can-dev openarm-can-utils

# makes up can0/can1 at 5 Mbps CAN FD data phase
openarm-can-cli can_configure     

# checks if can0 can1 are up
ip link show                         
openarm-can-cli -i can0 set_zero --arm
openarm-can-cli -i can1 set_zero --arm

