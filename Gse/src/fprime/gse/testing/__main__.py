'''
F' GDS Testing Main

This module allows the user to run the F' test case using F' style unittest symantics. Usually,
a user runs python unittests using the following command:

python -m unittest <test module/suite>

This can be replaced with the following command:

python -m fprime.gse.testing <test module/suite>

@author mstarch
'''
from __future__ import print_function
import os
import sys
import errno
import argparse
import datetime
import unittest

import fprime.gse.testing.runner
import fprime.gse.testing.support

def arg_parser(log_dir):
    '''
    Sets up the argument parsing for this module
    @param log_dir: log directory default
    '''
    parser = argparse.ArgumentParser(description="Fprime/unittest integration test run program. " +
                                     "used to setup standard logging, enable F' features and " +
                                     "provides an interactive mode used to step through tests.")
    parser.add_argument("-i", '--interactive', help='Run in interacive test mode', default=False,
                        action="store_true")
    parser.add_argument("-n", '--no-color', help='Run without colorizer', default=False,
                        action="store_true")
    parser.add_argument("-l", '--log-dir', help='Set logging directory', default=log_dir)
    parser.add_argument("-t", '--test-bed', help='Testbed value enabling testbed dependent tests',
                        default="default")
    parser.add_argument("-c", '--config', help='Configuration key value to add to testbed config',
                        nargs=2, action="append", required=False)
    parser.add_argument("--verbose", "-v", action='count')
    parser.add_argument("test")
    return parser.parse_args()

def main():
    '''
    Main function, used to provide a relatively simple interface to the FPrimeTestRunner enabling
    standard python unittest runs and an interactive mode for walking through tests.
    '''
    #Setup a log directory, and the use it as a default when parsing the arguments
    log_dir = os.path.join(os.environ.get("BUILD_ROOT", "/tmp"), "logging", "unittest")
    args = arg_parser(log_dir)

    #Prepare a timestamped log directory and file. Creating if necessary
    now = datetime.datetime.strftime(datetime.datetime.now(), "%Y-%m-%dT%H:%M:%S")
    try:
        os.makedirs(args.log_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(args.log_dir):
            raise
    #Setup testbed configuration directory
    tb_config = {
        "name": args.test_bed,
    }
    if args.config != None:
        for key, val in args.config:
            tb_config[key] = val
    #Open up a logging test runner
    with open(os.path.join(args.log_dir, now + "." + args.test), "w") as log:
        forker = fprime.gse.testing.support.StreamForker([sys.stderr, log])
        trunner = fprime.gse.testing.runner.FPrimeTestRunner(verbosity=args.verbose,
                                                             stream=forker,
                                                             interactive=args.interactive,
                                                             testbed=tb_config,
                                                             no_color=args.no_color)
        #Run using standard pyunit
        argsv = [sys.argv[0], args.test]
        unittest.main(module=None, testRunner=trunner, verbosity=args.verbose, argv=argsv)

if __name__ == "__main__":
    main()
