#!/bin/bash
# Uncomment this if you have not already cloned the repo
#git clone https://github.com/ConsolinnoEnergy/linux-mainline-phytec
#cd linux-mainline-phytec
#git checkout v5.4.y-phy-1u0022

# Get dockcross build environment 
docker run --rm dockcross/linux-armv7:latest > /tmp/dockcross
chmod +x /tmp/dockcross
# Define build instructions 
cat <<EOF > module_build.sh 
#!/bin/bash
sudo apt update && sudo apt install -y  libssl-dev 
make distclean
cp arch/arm/configs/imx6ull_consolinno_basemodule_defconfig .config 
# This updates the config with default values for undefined new features
make olddefconfig 
make scripts prepare modules_prepare
make -C . M=drivers/gpio
EOF
chmod +x  module_build.sh
# Run build in dockcross environment
/tmp/dockcross bash -c "./module_build.sh"
echo ".ko files are in modules are in drivers/gpio"