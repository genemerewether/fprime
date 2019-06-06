#!/bin/sh

set -e

cd DSPRELAY
make gen_make
make
cd ../
tar -czvf DSPRELAY.tar.gz \
    DSPRELAY/dspal-hex-clang-cross-opt-dspal-bin/DSPRELAY

cd MINRPC
make gen_make
make
cd ../
cd TESTRPC
make gen_make
make
cd ../
cd HEXREF/Top
make clean
cd ..
make
make dict_install rosser
cd Rpc
make ut_nocov_SDFLIGHT
cd ../..
tar -czvf HEXREF.tar.gz \
    HEXREF/py_dict \
    MINRPC/dspal-hex-clang-cross-opt-dspal-bin/MINRPC \
    TESTRPC/dspal-hex-clang-cross-opt-dspal-bin/TESTRPC \
    HEXREF/Rpc/test/ut/linux-linaro-cross-arm-ut-nocov-gnu-bin/test_ut \
    HEXREF/dspal-hex-clang-cross-opt-dspal-bin/HEXREF

cd R5REF/Top
make clean
cd ..
make
make dict_install rosser
cd ..
tar -czvf R5REF.tar.gz \
    R5REF/py_dict \
    R5REF/tir5-nortos-tms570lc43x-debug-opt-ccs7.0-bin/R5REF.out

cd SDREF/Top
make clean
cd ..
make
make dict_install rosser
cd ..
tar -czvf SDREF.tar.gz \
    SDREF/py_dict \
    SDREF/linux-linaro-cross-arm-opt-gnu-bin/SDREF

cd BLIMPREF/Top
make clean
cd ..
make
make dict_install rosser
cd ..
tar -czvf BLIMPREF.tar.gz \
    BLIMPREF/py_dict \
    BLIMPREF/*PrmDb.dat \
    BLIMPREF/linux-linaro-cross-arm-opt-gnu-bin/BLIMPREF

cd CARREF/Top
make clean
cd ..
make
make dict_install rosser
cd ..
tar -czvf CARREF.tar.gz \
    CARREF/py_dict \
    CARREF/*PrmDb.dat \
    CARREF/linux-linaro-cross-arm-opt-gnu-bin/CARREF
