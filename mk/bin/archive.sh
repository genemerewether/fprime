#!/bin/sh

set -e

cd TESTRPC
make
cd ../
cd HEXREF/Top
make clean
cd ..
make
make dict_install
cd Rpc
make ut_nocov_SDFLIGHT
cd ../..
touch Gse/generated/HEXREF/serializable/ROS/__init__.py
tar -czvf HEXREF.tar.gz \
    Gse/generated/HEXREF \
    TESTRPC/dspal-hex-clang-cross-opt-dspal-bin/TESTRPC \
    HEXREF/Rpc/test/ut/linux-linaro-cross-arm-ut-nocov-gnu-bin/test_ut \
    HEXREF/dspal-hex-clang-cross-opt-dspal-bin/HEXREF

cd SIMREF/Top
make clean
cd ..
make
make dict_install
cd ..
touch Gse/generated/SIMREF/serializable/ROS/__init__.py
tar -czvf SIMREF.tar.gz \
    Gse/generated/SIMREF \
    SIMREF/*PrmDb.dat \
    SIMREF/linux-linux-x86-debug-gnu-bin/SIMREF

cd R5REF/Top
make clean
cd ..
make
make dict_install
cd ..
touch Gse/generated/R5REF/serializable/ROS/__init__.py
tar -czvf R5REF.tar.gz \
    Gse/generated/R5REF \
    R5REF/tir5-nortos-tms570lc43x-debug-opt-ccs7.0-bin/R5REF.out

cd SDREF/Top
make clean
cd ..
make
make dict_install
cd ..
touch Gse/generated/SDREF/serializable/ROS/__init__.py
tar -czvf SDREF.tar.gz \
    Gse/generated/SDREF \
    SDREF/linux-linaro-cross-arm-opt-gnu-bin/SDREF
