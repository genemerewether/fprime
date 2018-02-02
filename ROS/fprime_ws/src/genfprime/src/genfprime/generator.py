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

from . name import _msg_serializable_xml_name, _srv_serializable_xml_name, _port_xml_name

import sys
sys.path.append('/Users/mereweth/quest/ROS/fprime_ws/devel/lib/python2.7/site-packages')
sys.path.append('/opt/ros/kinetic/lib/python2.7/site-packages')
sys.path.append('/Users/mereweth/quest/ROS/fprime_ws/src/genfprime/src/genfprime')
from name import _msg_serializable_xml_name, _srv_serializable_xml_name, _port_xml_name
import genmsg.command_line
msg_context = MsgContext.create_default()
_file = '/Users/mereweth/quest/ROS/fprime_ws/src/common_msgs/geometry_msgs/msg/PoseStamped.msg'
generate('geometry_msgs',
         _file,
         '/Users/mereweth/quest/ROS/Gen/geometry_msgs',
         genmsg.command_line.includepath_to_dict(dict(std_msgs='/Users/mereweth/quest/ROS/fprime_ws/src/std_msgs/msg',
                                                      geometry_msgs='/Users/mereweth/quest/ROS/fprime_ws/src/common_msgs/geometry_msgs')))

_file = '/Users/mereweth/quest/ROS/fprime_ws/src/common_msgs/diagnostic_msgs/srv/AddDiagnostics.srv'
generate('diagnostic_msgs',
         _file,
         '/Users/mereweth/quest/ROS/Gen/diagnostic_msgs',
         genmsg.command_line.includepath_to_dict(dict(std_msgs='/Users/mereweth/quest/ROS/fprime_ws/src/std_msgs/msg',
                                                      diagnostic_msgs='/Users/mereweth/quest/ROS/fprime_ws/src/common_msgs/diagnostic_msgs/srv')))

def generate(pkg, _file, out_dir, search_path):
    name = os.path.splitext(os.path.basename(_file))[0]

    if _file.endswith('.msg'):
        spec = genmsg.msg_loader.load_msg_from_file(msg_context, _file, name)
        with open(out_dir + os.path.sep + 'Types' + os.path.sep
                  + _msg_serializable_xml_name(name), 'w') as f:
            f.write("%s\n%s\n%s\n"%(pkg,_file,search_path))
            #gen_serializable_xml(spec)

    elif _file.endswith('.srv'):
        spec = genmsg.msg_loader.load_srv_from_file(msg_context, _file, name)

        with open(out_dir + os.path.sep + 'Types' + os.path.sep
                  + _srv_serializable_xml_name(name, is_input=True), 'w') as f:
            f.write("%s\n%s\n%s\n"%(pkg,_file,search_path))
            #gen_serializable_xml(spec.request)

        with open(out_dir + os.path.sep + 'Types' + os.path.sep
                  + _srv_serializable_xml_name(name, is_input=False), 'w') as f:
            f.write("%s\n%s\n%s\n"%(pkg,_file,search_path))
            #gen_serializable_xml(spec.response)

    with open(out_dir + os.path.sep + 'Ports' + os.path.sep
              + _port_xml_name(name), 'w') as f:
        f.write("%s\n%s\n%s\n"%(pkg,_file,search_path))
        #gen_port_xml(is_srv=(True if _file.endswith('.srv') else False))

    return spec

def gen_serializable_xml(spec):
    # TODO(mereweth) spec.constants
    root = etree.Element("interface", name="", namespace="")
    <serializable namespace="Fw" name="Buffer">
    <import_serializable_type>Path/To/SerializableAi.xml</import_serializable_type>
    <members>
      <member
        name="managerID"
        type="U32"
      />

      type="Namespace::Type"

def gen_port_xml(spec, is_srv)
    root = etree.Element("interface", name="", namespace="")
    <interface name="Ping" namespace="Svc">
    <import_serializable_type>Path/To/SerializableAi.xml</import_serializable_type>
        <args>
            <arg name="key" type="U32">
            type="Namespace::Type"
The XML tag name of elements is accessed through the tag property:

>>> print(root.tag)
root
Elements are organised in an XML tree structure. To create child elements and add them to a parent element, you can use the append() method:

>>> root.append( etree.Element("child1") )
However, this is so common that there is a shorter and much more efficient way to do this: the SubElement factory. It accepts the same arguments as the Element factory, but additionally requires the parent as first argument:

>>> child2 = etree.SubElement(root, "child2")
>>> child3 = etree.SubElement(root, "child3")

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
