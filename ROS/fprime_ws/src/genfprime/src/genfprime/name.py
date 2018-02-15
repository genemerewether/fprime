import os
#import genmsg.SEP

BASE_NAMESPACE = "ROS"
SEP = os.path.sep

def _include_msg_serializable_path(pkg, type_name):
    return '%s/Gen/%s/Types/%s'%(BASE_NAMESPACE,
                                 pkg,
                                 _msg_serializable_xml_name(type_name))

def _include_srv_serializable_path(pkg, type_name, is_input):
    return '%s/Gen/%s/Types/%s'%(BASE_NAMESPACE,
                                 pkg,
                                 _srv_serializable_xml_name(type_name,
                                                            is_input))

## :param type_name str: Name of message type sans package,
## e.g. 'String'
## :returns str: name of Serializable
def _msg_serializable_xml_name(type_name):
    return type_name + "SerializableAi.xml"

## :param type_name str: Name of message type sans package,
## e.g. 'String'
## :returns str: name of Port
def _port_xml_name(type_name):
    return type_name + "PortAi.xml"

## :param type_name str: Name of service type sans package,
## e.g. 'String'
## :param type_name bool: Is the type the argument side of service,
## e.g. 'True' or 'False'. Otherwise, we are returning the
## specified type
## :returns str: name of Serializable
def _srv_serializable_xml_name(type_name, is_input):
    if is_input:
        return type_name + "InSerializableAi.xml"
    else:
        return type_name + "OutSerializableAi.xml"

def _type_to_namespace(type_):
    if SEP in type_:
        split = type_.split(SEP)
        return "%s::%s::%s"%(BASE_NAMESPACE, split[0], split[1])
    else:
        return type_
