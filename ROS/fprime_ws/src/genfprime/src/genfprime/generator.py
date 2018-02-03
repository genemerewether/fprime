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

from . name import _msg_serializable_xml_name, _srv_serializable_xml_name
from . name import _port_xml_name, _type_to_namespace, BASE_NAMESPACE

from . types import msg_field_to_fprime


# import sys
# sys.path.append('/Users/mereweth/quest/ROS/fprime_ws/devel/lib/python2.7/site-packages')
# sys.path.append('/opt/ros/kinetic/lib/python2.7/site-packages')
# sys.path.append('/Users/mereweth/quest/ROS/fprime_ws/src/genfprime/src/genfprime')
# from name import _msg_serializable_xml_name, _srv_serializable_xml_name, _port_xml_name, _type_to_namespace, BASE_NAMESPACE
# import genmsg.command_line
# msg_context = MsgContext.create_default()
# _file = '/Users/mereweth/quest/ROS/fprime_ws/src/common_msgs/geometry_msgs/msg/PoseStamped.msg'
# generate('geometry_msgs',
#          _file,
#          '/Users/mereweth/quest/ROS/Gen/geometry_msgs',
#          genmsg.command_line.includepath_to_dict(dict(std_msgs='/Users/mereweth/quest/ROS/fprime_ws/src/std_msgs/msg',
#                                                       geometry_msgs='/Users/mereweth/quest/ROS/fprime_ws/src/common_msgs/geometry_msgs')))
#
# _file = '/Users/mereweth/quest/ROS/fprime_ws/src/common_msgs/diagnostic_msgs/srv/AddDiagnostics.srv'
# generate('diagnostic_msgs',
#          _file,
#          '/Users/mereweth/quest/ROS/Gen/diagnostic_msgs',
#          genmsg.command_line.includepath_to_dict(dict(std_msgs='/Users/mereweth/quest/ROS/fprime_ws/src/std_msgs/msg',
#                                                       diagnostic_msgs='/Users/mereweth/quest/ROS/fprime_ws/src/common_msgs/diagnostic_msgs/srv')))

def compute_type(pkg, field):
    return msg_field_to_fprime(field)

def generate(pkg, _file, out_dir, search_path):
    msg_context = MsgContext.create_default()
    name = os.path.splitext(os.path.basename(_file))[0]

    if _file.endswith('.msg'):
        spec = genmsg.msg_loader.load_msg_from_file(msg_context, _file, name)
        with open(out_dir + os.path.sep + 'Types' + os.path.sep
                  + _msg_serializable_xml_name(name), 'w') as f:
            f.write("<!--\n%s\n%s\n%s\n-->\n"%(pkg,_file,search_path))
            tree = gen_serializable_xml(spec, pkg)
            tree.write(f, pretty_print=True)

    elif _file.endswith('.srv'):
        spec = genmsg.msg_loader.load_srv_from_file(msg_context, _file, name)

        with open(out_dir + os.path.sep + 'Types' + os.path.sep
                  + _srv_serializable_xml_name(name, is_input=True), 'w') as f:
            f.write("<!--\n%s\n%s\n%s\n-->\n"%(pkg,_file,search_path))
            tree = gen_serializable_xml(spec.request, pkg)
            tree.write(f, pretty_print=True)

        with open(out_dir + os.path.sep + 'Types' + os.path.sep
                  + _srv_serializable_xml_name(name, is_input=False), 'w') as f:
            f.write("<!--\n%s\n%s\n%s\n-->\n"%(pkg,_file,search_path))
            tree = gen_serializable_xml(spec.response, pkg)
            tree.write(f, pretty_print=True)

    with open(out_dir + os.path.sep + 'Ports' + os.path.sep
              + _port_xml_name(name), 'w') as f:
        f.write("<!--\n%s\n%s\n%s\n-->\n"%(pkg,_file,search_path))
        #gen_port_xml(spec, pkg, is_srv=(True if _file.endswith('.srv') else False))

    return 0

def gen_serializable_xml(spec, pkg):
    # TODO(mereweth) spec.constants

    # TODO(mereweth) - is there ever a difference between spec full and short names?
    root = etree.Element("serializable",
                         name=spec.full_name,
                         namespace="%s::%s"%(BASE_NAMESPACE, pkg))

    import_header = etree.SubElement(root, "import_header")
    import_header.text = 'Fw/Time/Time.hpp'
    import_header = etree.SubElement(root, "import_header")
    import_header.text = 'Fw/Types/EightyCharString.hpp'

    # etree.SubElement(root, "import_serializable_type",
    #                  'Fw/Time/TimeSerializableAi.xml')
    # TODO(mereweth) - add imports for dependencies
    #<import_serializable_type>Path/To/SerializableAi.xml</import_serializable_type>
    #if msg_context.is_registered(type_):
    members = etree.SubElement(root, "members")

    # TODO(mereweth) - what is field.type vs field.base_type?
    for field in spec.parsed_fields():
        # TODO(mereweth) - decide how to handle arrays besides just capping size
        type_, size_, imports = compute_type(pkg, field)

        #TODO(mereweth) change to 'if size_ is not None' if we want to make some arrays into bigger, non-array data types
        if field.is_array and size_ is None:
            array_len_spec = field.array_len
            if array_len_spec is None:
                raise MsgGenerationException("No calc size for array field: %s, msg: %s, pkg: %s"%(
                                             field.name, spec.full_name, pkg))
            else:
                raise MsgGenerationException("No calc size for array len: %d, field: %s, msg: %s, pkg: %s"%(
                                             array_len_spec,
                                             field.name, spec.full_name, pkg))

            member = etree.SubElement(members,
                                      "member",
                                      name=field.name,
                                      type=type_,
                                      size=str(size_))

        else:
            member = etree.SubElement(members,
                                      "member",
                                      name=field.name,
                                      type=type_)

    return root.getroottree()

# def gen_port_xml(spec, pkg, is_srv)
#     root = etree.Element("interface", name="", namespace="ROS")
#     <interface name="Ping" namespace="Svc">
#     <import_serializable_type>Path/To/SerializableAi.xml</import_serializable_type>
#         <args>
#             <arg name="key" type="U32">
#             type="Namespace::Type"
# The XML tag name of elements is accessed through the tag property:
#
# >>> print(root.tag)
# root
# Elements are organised in an XML tree structure. To create child elements and add them to a parent element, you can use the append() method:
#
# >>> root.append( etree.Element("child1") )
# However, this is so common that there is a shorter and much more efficient way to do this: the SubElement factory. It accepts the same arguments as the Element factory, but additionally requires the parent as first argument:
#
# >>> child2 = etree.SubElement(root, "child2")
# >>> child3 = etree.SubElement(root, "child3")





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
