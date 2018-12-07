#!/bin/env python
#===============================================================================
# NAME: ImageUpdater.py
#
# DESCRIPTION: This module is an image updater which uses the observer
#                pattern to update all instanced panels. This module will
#                utilize the logging module to create disk file logs of
#                image updates.
#
# AUTHOR: mereweth
# EMAIL:  mereweth@jpl.nasa.gov
# DATE CREATED: Nov 5, 2017
#
# Copyright 2017, California Institute of Technology.
# ALL RIGHTS RESERVED. U.S. Government Sponsorship acknowledged.
#===============================================================================
#
# Python standard modules
#
from __future__ import print_function
import sys
import os
import logging
import time

from fprime.gse.controllers import consumer
from fprime.gse.controllers import exceptions

from fprime.gse.utils import Logger

'''TODO(mereweth) - when more types of lossy telemetry are added, refactor this
to inherit from consumer.Consumer
Also write LossyThreadedTcpServer (network shaping?), lossy_socket_listener
Idea being that some types of telemetry are large and/or useless if delayed
'''
import Queue
import threading
HAS_PYZMQ = False
try:
    import zmq
    HAS_PYZMQ = True    
except ImportError, e:
    print("", file=sys.stderr)
    print(e, file=sys.stderr)
    print("Image streaming requires special installation:", file=sys.stderr)
    print("sudo -H pip install pyzmq\n", file=sys.stderr)
import PIL.Image
'''NOTE(mereweth) - end delete
'''


class ImageListener(consumer.Consumer):
    """
    A singleton class that will update all image widgets
    on all active instances of UI windows.  All the various
    objects within this program will use this to update
    status in the common image area of the panels.

    Also every update is saved in a default log file
    """
    __instance = None


    def __init__(self, context=None):
        """
        Constructor.
        WARNING: After the first instantiation setupLogging must be executed.
        """
        super(ImageListener, self).__init__()
        #
        # Variables used by standard Update interface
        #
        self.__update_mode = "Update"
        self.__opt = None

        self.__logger = None

        if HAS_PYZMQ:
            self.context = context or zmq.Context.instance()
        else:
            self.context = None

        setattr(self, "__update_mode", "Update")

        '''NOTE(mereweth) - when refactored, delete these attributes
        '''
        # Run flag
        self.__run_filler = False

        # Store thread handle
        self.__thread_filler = None
        '''NOTE(mereweth) - end delete
        '''

    def getInstance():
        """
        Return instance of singleton.
        """
        if(ImageListener.__instance is None):
            ImageListener.__instance = ImageListener()
        return ImageListener.__instance

    #define static method
    getInstance = staticmethod(getInstance)

    '''NOTE(mereweth) - when refactored, delete these methods
    '''
    def start_thread_filler(self, opt=None):        
        if not HAS_PYZMQ:
            return

        if opt.stream_port is None:
            print("No image streaming port specified", file=sys.stderr)
            return

        self.__run_filler = True

        img_sock = self.context.socket(zmq.SUB)
        img_sock.setsockopt(zmq.SUBSCRIBE, 'ZP')
        img_sock.setsockopt(zmq.LINGER, 0)
        img_sock.setsockopt(zmq.RCVTIMEO, 0)
        img_sock.set_hwm(1)
        try:
            #TODO(mereweth) - add option for bind vs connect?
            img_sock.bind("tcp://*:%d"%opt.stream_port)
        except Exception as e:
            print("Could not bind image streaming socket due to error %s"%e, file=sys.stderr)
            return
        self.__thread_filler = threading.Thread(target=self.fill_queue, args=[img_sock])
        self.__thread_filler.start()

    def stop_thread_filler(self):
        self.__run_filler = False
        if self.__thread_filler is not None:
            self.__thread_filler.join()

    def fill_queue(self, img_sock):
        while self.__run_filler:
            # if service_queue has fallen behind:
            if self.queue_size() > 1:
                continue
            try:
                img = img_sock.recv(flags=zmq.NOBLOCK)[2:] # strip ZMQ id
                time.sleep(0.05)
            except:
                time.sleep(0.1)
                continue
            # TODO(mereweth) - if timed out, clear image?
            try:
                #TODO(mereweth) - deserialize from port
                # TODO(mereweth) discard if too old?
                ptr = 0

                self.put_data(PIL.Image.frombytes('L', (640, 480), img))
                self.__log_info("Put %d bytes into queue"%len(img))
            except Exception as e:
                print("Unable to decode image due to error:\n%s"%e, file=sys.stderr)
    '''NOTE(mereweth) - end delete
    '''

    def setupLogging(self, opt=None):
        """
        Set up logging once for the singleton to use.
        """

        if opt == None:
            p = os.environ['HOME'] + os.sep + 'logs' + os.sep + "image"
        else:
            p = opt.log_file_path + os.sep + opt.log_file_prefix + os.sep + "image"
        #
        self.__opt = opt
        logfile = 'Image.log'
        #
        if not os.path.exists(p):
            os.makedirs(p)
        f = p + os.sep + logfile
        self.__logger = Logger.connectOutputLogger(f)
        self.__logger.info("Created log: %s" % f)
        self.__logger.info("User: %s" % os.environ['USER'])

    def __log_info(self, s):
        """
        Update log message with time tag.
        """
        if self.__logger is None:
            return
        if self.__opt is None or self.__opt.log_time == "local":
            t = time.strftime("%y-%m-%d %H:%M:%S:", time.localtime())
        else:
            t = time.strftime("%y-%m-%d %H:%M:%S:", time.gmtime())
        self.__logger.info("%s %s" % (t,s))

    def process_data(self, img):
        self.update_mode = 'Update'
        self.__log_info("Received image of type %s"%type(img))

    def clear(self):
        """
        Clear the image widget.
        """
        self.update_mode = 'Clear'
        self.notifyObservers(self.update_mode)

    #
    # Interfaces for observer update method to utilize

    @property
    def update_mode(self):
        return self.__update_mode

    @update_mode.setter
    def update_mode(self, u):
        self.__update_mode = u


if __name__ == "__main__":
    pass
