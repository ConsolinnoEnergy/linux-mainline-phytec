#!/bin/bash
sudo apt update && sudo apt install -y  libssl-dev 
make distclean
cp arch/arm/configs/imx6ull_consolinno_basemodule_defconfig .config 
# This updates the config with default values for undefined new features
make olddefconfig 
make scripts prepare modules_prepare
make -C . M=drivers/gpio
