#!/bin/sh

tar -czvf HEXREF.tar.gz \
    Gse/generated/HEXREF \
    TESTRPC/dspal-hex-clang-cross-opt-dspal-bin/TESTRPC \
    HEXREF/Rpc/test/ut/linux-linaro-cross-arm-ut-nocov-gnu-bin/test_ut \
    HEXREF/dspal-hex-clang-cross-opt-dspal-bin/HEXREF

tar -czvf SIMREF.tar.gz \
    Gse/generated/SIMREF \
    SIMREF/*PrmDb.dat \
    SIMREF/linux-linux-x86-debug-gnu-bin/SIMREF

tar -czvf R5REF.tar.gz \
    Gse/generated/R5REF \
    R5REF/tir5-nortos-tms570lc43x-debug-opt-ccs7.0-bin/R5REF

tar -czvf SDREF.tar.gz \
    Gse/generated/SDREF \
    SDREF/linux-linaro-cross-arm-opt-gnu-bin/SDREF
