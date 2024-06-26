#!/bin/bash

export TOOLS_PATH=/opt/gcc-arm-none-eabi
export ARM_VERSION=13.2.rel1
export ARM_ARCH=x86_64
echo "Downloading ARM GNU GCC for Platform: $ARM_ARCH"
mkdir ${TOOLS_PATH}

URL="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_VERSION}/binrel/arm-gnu-toolchain-${ARM_VERSION}-${ARM_ARCH}-arm-none-eabi.tar.xz"
echo "Downloading ARM GNU GCC from: $URL"

curl -Lo gcc-arm-none-eabi.tar.xz ${URL} \
  && tar xf gcc-arm-none-eabi.tar.xz --strip-components=1 -C ${TOOLS_PATH} \
	&& rm gcc-arm-none-eabi.tar.xz \
	&& rm ${TOOLS_PATH}/*.txt \
	&& rm -rf ${TOOLS_PATH}/share/doc \
	&& echo $URL

