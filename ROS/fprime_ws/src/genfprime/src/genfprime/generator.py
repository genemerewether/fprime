from __future__ import print_function

import os
import keyword
import itertools
import sys
import traceback
import struct
from lxml import etree

import genmsg
import genmsg.msgs
import genmsg.msg_loader
import genmsg.gentools

from genmsg import InvalidMsgSpec, MsgContext, MsgSpec, MsgGenerationException
from genmsg.base import log


# def generate_msg(pkg, files, out_dir, search_path):
#     """
#     Generate F Prime XML for all messages in a package
#     """
#     msg_context = MsgContext.create_default()
#     for f in files:
#         f = os.path.abspath(f)
#         infile = os.path.basename(f)
#         full_type = genmsg.gentools.compute_full_type_name(pkg, infile)
#         spec = genmsg.msg_loader.load_msg_from_file(msg_context, f, full_type)
#         generate_msg_from_spec(msg_context, spec, search_path, out_dir, pkg)

# def generate_srv(pkg, files, out_dir, search_path):
#     """
#     Generate F Prime XML for all services in a package
#     """
#     msg_context = MsgContext.create_default()
#     for f in files:
#         f = os.path.abspath(f)
#         infile = os.path.basename(f)
#         full_type = genmsg.gentools.compute_full_type_name(pkg, infile)
#         spec = genmsg.msg_loader.load_srv_from_file(msg_context, f, full_type)
#         generate_srv_from_spec(msg_context, spec, search_path, out_dir, pkg, f)

# def generate_msg_from_spec(msg_context, spec, search_path, output_dir, package, msgs=None):
#     """
#     Generate a message

#     @param msg_path: The path to the .msg file
#     @type msg_path: str
#     """
#     genmsg.msg_loader.load_depends(msg_context, spec, search_path)
#     spec.actual_name=spec.short_name
#     spec.component_type='message'
#     msgs = msg_list(package, search_path, '.msg')
#     for m in msgs:
#         genmsg.load_msg_by_type(msg_context, '%s/%s'%(package, m), search_path)
