#!/bin/bash

set -e

SDFLIGHT_TOOLS_ROOT=/opt/tools/quest/
QRLINUX_SYSROOT_NAME=qrlinux_sysroot
UBUNTU_SYSROOT_NAME=indigo_sysroot

QUEST_DEV_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TEMP_STAGE_SYSROOT=${QUEST_DEV_ROOT}/cross_toolchain/downloads/temp_stage_sysroot
export UBUNTU_SYSROOT=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/${UBUNTU_SYSROOT_NAME}/

sudo mkdir -p $SDFLIGHT_TOOLS_ROOT
sudo chown $USER $SDFLIGHT_TOOLS_ROOT

REINSTALL=1

if [ $REINSTALL ]; then
# Install software

sudo apt-get install -y python-pip python3-pip xterm csh cmake python-empy xauth colormake tree

# snapdragon flight
sudo apt-get install -y android-tools-adb android-tools-fastboot  \
     fakechroot fakeroot unzip xz-utils qemu-user-static qemu-system-arm \
     proot

sudo apt-get install -y lib32stdc++6 diffstat texi2html texinfo subversion \
    chrpath gettext

# speed up compilation
sudo apt-get install -y ccache

sudo apt-get install -y python-catkin-tools || sudo apt-get install -y catkin
fi

pushd .

if [ -e ${QUEST_DEV_ROOT}/cross_toolchain ]; then
    cd ${QUEST_DEV_ROOT}/cross_toolchain

    export HEXAGON_SDK_ROOT=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/Hexagon_SDK/3.0
    ./installsdk.sh --APQ8074 --arm-gcc --qrlSDK --no-verify ${SDFLIGHT_TOOLS_ROOT}
    mv ${SDFLIGHT_TOOLS_ROOT}/Qualcomm/ARM_Tools/gcc-4.9-2014.11 ${SDFLIGHT_TOOLS_ROOT}/Qualcomm/Hexagon_SDK/3.0/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux || true
    cd ${SDFLIGHT_TOOLS_ROOT}/Qualcomm/Hexagon_SDK/3.0/libs/common/rpcmem
    make V=UbuntuARM_Debug

    export HEXAGON_ARM_SYSROOT=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/${QRLINUX_SYSROOT_NAME}/

    cd ${QUEST_DEV_ROOT}
    if [ ! -f cross_toolchain/downloads/linaro-trusty-developer-20140922-682.tar.gz ]; then
        wget -P cross_toolchain/downloads http://releases.linaro.org/archive/14.09/ubuntu/trusty-images/developer/linaro-trusty-developer-20140922-682.tar.gz
    fi
    if [ ! -f ${HEXAGON_ARM_SYSROOT}/SYSROOT_UNPACKED ]; then
        mkdir -p ${TEMP_STAGE_SYSROOT}
        echo "Unpacking sysroot to temporary staging location ${TEMP_STAGE_SYSROOT}"
        tar -C ${TEMP_STAGE_SYSROOT} --strip-components=1 --exclude="dev/*" -xzf cross_toolchain/downloads/linaro-trusty-developer-20140922-682.tar.gz && echo "Linaro Trusty sysroot unpacked on" `date` > ${TEMP_STAGE_SYSROOT}/SYSROOT_UNPACKED

        export PROOT_NO_SECCOMP=1 # see https://github.com/proot-me/PRoot/issues/106
        #rm ${HEXAGON_ARM_SYSROOT}/lib/libc-2.17-2013.07-2.so
        rm -f ${TEMP_STAGE_SYSROOT}/etc/resolv.conf
        cp /etc/resolv.conf ${TEMP_STAGE_SYSROOT}/etc/
        proot -b ${QUEST_DEV_ROOT}:/home/linaro/tmp -S ${TEMP_STAGE_SYSROOT} -q qemu-arm-static -w /home/linaro/tmp /bin/bash trusty_bootstrap.bash

        #rsync -l -r ${TEMP_STAGE_SYSROOT}/ ${HEXAGON_ARM_SYSROOT}
        mkdir -p ${UBUNTU_SYSROOT}
        mv ${TEMP_STAGE_SYSROOT}/* ${UBUNTU_SYSROOT}/
	
        # skip the root directory and /host-rootfs to avoid messing up the host filesystem
        #proot -0 -r ${HEXAGON_ARM_SYSROOT} -q qemu-arm-static -w /home/linaro symlinks -cr `find /?* -maxdepth 0 | grep -v 'host-rootfs'`
        proot -0 -r ${UBUNTU_SYSROOT} -q qemu-arm-static -w /home/linaro /usr/bin/symlinks -cr `find /?* -maxdepth 0 | grep -v 'host-rootfs'`

        # pthread ld script uses absolute paths in this sysroot, which breaks cross-compile
        PTHREAD_FILE=/usr/lib/arm-linux-gnueabihf/libpthread.so
        echo "/* GNU ld script" > ${UBUNTU_SYSROOT}/${PTHREAD_FILE}
        echo "Use the shared library, but some functions are only in" >> ${UBUNTU_SYSROOT}/${PTHREAD_FILE}
        echo "the static library, so try that secondarily.  */" >> ${UBUNTU_SYSROOT}/${PTHREAD_FILE}
        echo "OUTPUT_FORMAT(elf32-littlearm)" >> ${UBUNTU_SYSROOT}/${PTHREAD_FILE}
        echo "GROUP ( libpthread.so.0 libpthread_nonshared.a )" >> ${UBUNTU_SYSROOT}/${PTHREAD_FILE}

        LIBC_FILE=/usr/lib/arm-linux-gnueabihf/libc.so
        echo "/* GNU ld script" > ${UBUNTU_SYSROOT}/${LIBC_FILE}
        echo "Use the shared library, but some functions are only in" >> ${UBUNTU_SYSROOT}/${LIBC_FILE}
        echo "the static library, so try that secondarily.  */" >> ${UBUNTU_SYSROOT}/${LIBC_FILE}
        echo "OUTPUT_FORMAT(elf32-littlearm)" >> ${UBUNTU_SYSROOT}/${LIBC_FILE}
        echo "GROUP ( ../../../lib/arm-linux-gnueabihf/libc.so.6 ../../../usr/lib/arm-linux-gnueabihf/libc_nonshared.a" >> ${UBUNTU_SYSROOT}/${LIBC_FILE}
        echo "AS_NEEDED ( ../../../lib/arm-linux-gnueabihf/ld-linux-armhf.so.3 ))" >> ${UBUNTU_SYSROOT}/${LIBC_FILE}
    fi
else
    echo "cross_toolchain not found; skipping"
fi

cd $HOME
mkdir bin
curl https://storage.googleapis.com/git-repo-downloads/repo > ./bin/repo
chmod a+x ./bin/repo

cd $HOME
git clone https://github.com/anestisb/android-simg2img.git
cd android-simg2img
make

exportlines="export HEXAGON_SDK_ROOT=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/Hexagon_SDK/3.0/
export HEXAGON_TOOLS_ROOT=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/HEXAGON_Tools/7.2.12/Tools/
export HEXAGON_ARM_SYSROOT=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/${QRLINUX_SYSROOT_NAME}/
export UBUNTU_SYSROOT=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/${UBUNTU_SYSROOT_NAME}/
export ARM_CROSS_GCC_ROOT=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/ARM_Tools/gcc-4.9-2014.11
export PATH=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/Hexagon_SDK/3.0/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin:$PATH
export PATH=${HOME}/android-simg2img:${HOME}/bin:\$PATH
alias mdm=${SDFLIGHT_TOOLS_ROOT}/Qualcomm/Hexagon_SDK/3.0/tools/debug/mini-dm/Linux_Debug/mini-dm
cd ${QUEST_DEV_ROOT}"

for exportline in "$exportlines"
do
    if grep -Fxq "$exportline" ~/.bashrc; then
	echo "nothing to do for $exportline"
    else
	echo "$exportline" >> ~/.bashrc
	echo -e "\n" >> ~/.bashrc
    fi
    if grep -Fxq "$exportline" ~/.zshrc; then
	echo "nothing to do for $exportline"
    else
	echo "$exportline" >> ~/.zshrc
	echo -e "\n" >> ~/.zshrc
    fi
done

. ~/.bashrc
popd

# Configure hardware related bits
sudo apt-get -y remove modemmanager
sudo usermod -a -G dialout $USER

# configure permissions for adb
cd ${QUEST_DEV_ROOT}
sudo cp adb_devices /etc/udev/rules.d/51-android.rules
sudo chmod a+r /etc/udev/rules.d/51-android.rules
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
