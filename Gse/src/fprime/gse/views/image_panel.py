#!/bin/env python
#===============================================================================
# NAME: ImagePanel.py
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

import sys
import Tkinter
import PIL

HAS_PIL = False
try:
    import PIL.ImageTk
    HAS_PIL = True
except ImportError, e:
    sys.stderr.write("\n" + str(e))
    sys.stderr.write("\nImage streaming requires special installation:")
    sys.stderr.write("\nsudo apt-get install python-pil.imagetk")
    
    sys.stderr.write("\nOn some older systems, sudo apt-get install python-imaging-tk\n\n")

from fprime.gse.controllers import exceptions
from fprime.gse.controllers import observer

class ImagePanel(observer.Observer):
    '''
    A class that instances and communicates with an image display panel
    '''
    def __init__(self, parent, top):
        '''
        Constructor
        '''
        # The parent notebook widget
        self.__parent = parent
        # The main top widget with the menus
        self.__top = top

        self.__image = None

        self.__display = Tkinter.Label(parent)
        self.__display.pack(side=Tkinter.TOP, expand=Tkinter.YES, fill=Tkinter.BOTH)

    # Interface for updating image via observer singleton
    def update(self, observer, img):
        """
        Update the image widget for each
        instanced main panel.  This is only called
        from the image_listener singleton observer
        class.
        """

        if observer.update_mode == "Update":
            self.imageUpdate(img)
        elif observer.update_mode == "Clear":
            self.imageClear()
        else:
            raise exceptions.GseControllerImageUpdateException(observer.update_mode)

    def imageUpdate(self, img):
        """
        Update image on the widget.
        """
        ''' TODO(mereweth) - should we store the image locally? When it is
        garbage-collected, it will disappear from the Tk Label
        '''
        
        if not HAS_PIL:
            return

        try:
            self.__image = PIL.ImageTk.PhotoImage(img)
        except Exception as e:
            print("Error updating image: %s\n"%e)
        else:
            self.__display.config(image=self.__image)
            self.__parent.update()


    def imageClear(self):
        """
        Clear the image widget.
        """
        self.__display.clear()
