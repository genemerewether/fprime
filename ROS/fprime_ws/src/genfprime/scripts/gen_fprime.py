#!/usr/bin/env python

"""
ROS service Serializable and Port XML generation for FPrime.

Converts ROS .msg and .srv files into FPrime descriptions.
"""
import os
import sys

#import genfprime.generator
import genfprime.genfprime_main

if __name__ == "__main__":
    genfprime.genfprime_main.genmain(sys.argv, 'gen_fprime.py')
    
