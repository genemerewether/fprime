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


def compute_type(pkg, field):
    return msg_field_to_fprime(pkg, field)

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

    members = etree.SubElement(root, "members")

    if len(spec.parsed_fields()) == 0:
        # NOTE(mereweth) Empty.msg, possibly others
        member = etree.SubElement(members,
                                  "member",
                                  name="unused",
                                  type='U8')

    # TODO(mereweth) - what is field.type vs field.base_type?
    for field in spec.parsed_fields():
        # TODO(mereweth) - decide how to handle arrays besides just capping size
        type_, size_, header_includes, serial_imports = compute_type(pkg, field)

        for header in header_includes:
            include_header = etree.SubElement(root, "include_header")
            include_header.text = header

        for serial in serial_imports:
            import_serial = etree.SubElement(root, "import_serializable_type")
            import_serial.text = serial

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
