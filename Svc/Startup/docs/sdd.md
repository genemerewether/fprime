<title>Svc::Startup Component SDD</title>
# Svc::Startup Component

## 1. Introduction

The `Svc::Startup` component is responsible for starting FSW upon boot, and for
recording events that happen before FSW is up.

## 2. Requirements

Requirement | Description | Verification Method
|:---|:---|:---|
SVC-STARTUP-001 | The `Startup` scripts shall attempt to run FSW binaries in the following order: 1) "Run-once" image; 2) Currently selected image; 3) Validated Golden FSW image 4) All three Golden FSW images in turn if all 3 fail CRC check | Unit Test
SVC-STARTUP-002 | The `Startup` scripts shall allow selection of the current FSW image and store this selection through reboots | Unit Test
SVC-STARTUP-003 | The `Startup` scripts shall store three copies of the CRC with each unique FSW image. | Unit Test
SVC-STARTUP-004 | The `Startup` scripts shall store three copies of each FSW image. | Unit Test
SVC-STARTUP-005 | The `Startup` scripts shall allow running a FSW binary if and only if at least one of the copies of that binary matches the CRC stored with it. | Unit Test
SVC-STARTUP-006 | If only two of the three copies of a given CRC match, the `Startup` scripts shall take the two that match as truth. | Unit Test
SVC-STARTUP-007 | After attempting to run the "run-once" image, the `Startup` scripts shall remove the symlink (but not the image), whether the run was successful or not. | Unit Test
SVC-STARTUP-008 | The `Startup` scripts shall archive core dump files generated in the specified directory when the `Startup` scripts are next run. | Unit Test
SVC-STARTUP-009 | The `Startup` scripts shall capture and archive stdout and stderr from the running FSW binary to the specified directory. | Unit Test
SVC-STARTUP-010 | Upon termination of a FSW binary, the `Startup` scripts shall start the next binary in the order specified in SVC-STARTUP-001. | Unit Test
SVC-STARTUP-011 | The `Startup` scripts shall be located on a read-only partition to protect them from modification. | Unit Test
SVC-STARTUP-012 | The `Startup` scripts shall allow the root partition and the FSW partition to be read-only. | Unit Test
SVC-STARTUP-013 | The `Startup` scripts shall keep a boot counter to allow unique but predictable names for cores and stderr/stdout. | Unit Test
SVC-STARTUP-014 | The `Startup` scripts shall only run binaries that match the deployment name specified in the configuration file, plus per-copy unique suffix | Unit Test
SVC-STARTUP-015 | The `Startup` scripts produce the same CRC output as the FSW build system | Unit Test
SVC-STARTUP-016 | The `Startup` script magic cookie for halting FSW run shall NOT persist across reboot | Unit Test
SVC-STARTUP-017 | The `Startup` script and fstab shall still boot and run FSW even if any read-write partition is corrupted | Unit Test
SVC-STARTUP-018 | The `Startup` script shall read CRC files as plain ASCII text, unsigned decimal numbers, of the format read by `$(cat PATH_TO_CRC_FILE)` | Unit Test

## 3. Design

### 3.1 Context

### 3.1.1 `Svc::Startup` and `Nav::NavPatch`

`Svc::Startup` works with `Nav::NavPatch` to handle patching the Nav code.
Therefore, the requirements must be consistent between the two modules.

### 3.1.2 NAV Boot Sequence
1) Primary boot loader (ROM)
2) Secondary Boot loader (sbl partition)
    1) TrustZone (tz partition)
    2) RPM (rpm partition)
3) Application bootloader/LK (aboot partition)
4) Linux kernel and RAM disk (boot partition)
5) Upstart control system (Linux userspace)
6) `Svc::Startup`
7) FSW binary

### 3.2 Functional Description

### 3.2.1 Use of Linux Filesystem
`Svc::Startup` relies extensively on the Linux filesystem for loading FSW
binaries, maintaining symlinks to store the CURRENT and RUN_ONCE versions,
keeping a boot counter, logging `stderr` and `stdout` from the FSW binary,
and logging internal EVRs in turn.

### 3.2.2 Launch Control Agent
`Svc::Startup` was originally designed to work with the Upstart control system,
but could probably work with any system, as long as that system allows
backgrounding of a script. This is required because `Svc::Startup` blocks while
the FSW binaries are running. In particular, during the "hail mary" loop, if
all the binaries have failed checksumming, `Svc::Startup` cyclically runs all
three copies of the GOLDEN version, so it will never exit.

### 3.3 Scenarios

### 3.3.1 Flight Operations Scenario

**IMPORTANT FLIGHT RULE**

The design of `Svc::Startup` requires that the `CURRENT_SYMLINK` persist across
reboots securely. There is a risk that if a malformed binary is uploaded and
the `CURRENT_SYMLINK` updated to point to it, we could lose contact permanently
with the vehicle. The FPGA design is required to shut down the NAV processor
after some timeout, so even if the malformed binary would have crashed, the NAV
timeout will happen before that point and bring the board down before
`Svc::Startup` has a chance to run the GOLDEN version.

**Therefore, ALWAYS test an uploaded binary by setting ONLY the `RUN_ONCE`
symlink to point to the newly uploaded binary. After a reboot, once the FSW has
successfully come online, and the version has been confirmed to be the same as
the newly uploaded version (rather than the GOLDEN version coming online after
two reboots), then and only then can the `CURRENT_SYMLINK` be updated to point
to this new version.**

During flight, the FSW will always automatically start once the NAV processor
comes online.

### 3.3.2 Test Operations Scenario

During testing, there is a hook for preventing the FSW from starting, although
it should not be relied upon for safety-critical operations. Before powering on
the NAV processor, ensure by inspection / partial testing that the adb
connection will be successful once the NAV is booted. Issue the following
command: `leonardo-fsw/VNV/scripts/inhibitFSW.bash`, then power on the NAV.

Note - some commands for cleanly shutting down the system will trigger
the `filesystem` event, causing Upstart to start the `Svc::Startup` scripts
again. In testing, the board always shuts down before the FSW actually runs
(due to the MAGIC_COOKIE timeout), but it is something to be aware of.

### 3.4 State

`Svc::Startup` has no state machines.

### 3.5 Algorithms

`Svc::Startup` has no significant algorithms.

## 4. Dictionaries

N/A

## 5. Module Checklists

N/A

## 6. Unit Testing
The unit tests employ the BATS Bash unit testing framework. On Mac, use
`brew install bats`, and on Ubuntu 16.04, `apt-get install bats`. Otherwise,
you can install from source following:
https://github.com/bats-core/bats-core#installing-bats-from-source

Then, `make test` to run all tests. You can also change to the `test/ut`
directory and run `bats` with any of the `*.bats` files as the only argument.

## 6.1 Unit Test Descriptions

`test_boot_counter.bats`
- Test missing/malformed boot counter files

`test_hash_load_compare.bats`
- Test hash loading when folder symlink is missing, or destination of symlink is missing, or folder is not a symlink
- Test hash loading when checksums and/or binaries are missing, or when they don't match

`test_hash_parse_compare.bats`
- Test `cksum` output format parsing
- Test CRC voting

`test_logging.bats`
- Test that missing/malformed VERBOSITY override variable and file don't affect ability to issue EVRs

`test_startup_helpers.bats`
- Test appending the boot counter to the FSW args

`test_startup.bats`
- Integrated testing of the startup system
- Check various combinations of RUN_ONCE and CURRENT symlinks

## 6.2 Unit Test Output

## 7. Change Log

Date | Description
---- | -----------
4/18/2018 | Initial SDD
5/4/2018 | Complete documentation after code review
