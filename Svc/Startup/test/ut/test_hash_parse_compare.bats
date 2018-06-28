#!/usr/bin/env bats

. ./hash_validation.bash

############################# get_crc32_from_cksum #############################
@test "parse cksum output" {
  get_crc32_from_cksum dest_var 1714674347 518 settings.bash
  [ $dest_var -eq 1714674347 ]
}

@test "parse cksum output when file is missing" {
  ! get_crc32_from_cksum dest_var $(cksum DOES_NOT_EXIST)
}

################################### crc_vote ###################################
@test "crc voting all equal" {
  crc_vote dest_var 1714674347 1714674347 1714674347
  [ $dest_var -eq 1714674347 ]
}

@test "crc voting first two agree" {
  crc_vote dest_var 7146743 7146743 2
  [ $dest_var -eq 7146743 ]
}

@test "crc voting one and three agree" {
  crc_vote dest_var 474347 2 474347
  [ $dest_var -eq 474347 ]
}

@test "crc voting two and three agree" {
  crc_vote dest_var 3 4587402 4587402
  [ $dest_var -eq 4587402 ]
}

@test "crc voting none agree" {
  ! crc_vote dest_var 1 2 3
  [ $dest_var -eq -1 ]
}
