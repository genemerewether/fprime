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
    Gse/generated/HEXREF \
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
    Gse/generated/R5REF \
    R5REF/tir5-nortos-tms570lc43x-debug-opt-ccs7.0-bin/R5REF.out

cd SDREF/Top
make clean
cd ..
make
make dict_install rosser
cd ..
tar -czvf SDREF.tar.gz \
    Gse/generated/SDREF \
    SDREF/linux-linaro-cross-arm-opt-gnu-bin/SDREF

cd BLIMPREF/Top
make clean
cd ..
make
make dict_install rosser
cd ..
tar -czvf BLIMPREF.tar.gz \
    Gse/generated/BLIMPREF \
    BLIMPREF/*PrmDb.dat \
    BLIMPREF/linux-linaro-cross-arm-opt-gnu-bin/BLIMPREF

cd CARREF/Top
make clean
cd ..
make
make dict_install rosser
cd ..
tar -czvf CARREF.tar.gz \
    Gse/generated/CARREF \
    CARREF/*PrmDb.dat \
    CARREF/linux-linaro-cross-arm-opt-gnu-bin/CARREF
