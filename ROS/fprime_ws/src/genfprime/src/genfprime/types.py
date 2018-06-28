from . name import _type_to_namespace, BASE_NAMESPACE, SEP, _include_msg_serializable_path

MAX_ARRAY_LEN = 100

# User C++ Type defines (Found from BasicTypes.hpp)
# <value>int32_t</value>
# <value>uint32_t</value>
# <value>I8</value>
# <value>U8</value>
# <value>I16</value>
# <value>U16</value>
# <value>I32</value>
# <value>U32</value>
# <value>I64</value>
# <value>U64</value>
# <value>F32</value>
# <value>F64</value>
# <value>NATIVE_INT_TYPE</value>
# <value>NATIVE_UINT_TYPE</value>
# <value>string</value>

MSG_TYPE_TO_FPRIME = {'byte': 'I8',
                      'char': 'U8',
                      'bool': 'U8',
                      'uint8': 'U8',
                      'int8': 'I8',
                      'uint16': 'U16',
                      'int16': 'I16',
                      'uint32': 'U32',
                      'int32': 'I32',
                      'uint64': 'U64',
                      'int64': 'I64',
                      'float32': 'F32',
                      'float64': 'F64',
                      'string': 'Fw::EightyCharString',
                      'time': 'Fw::Time',
                      'duration': 'Fw::Time'}

MSG_TYPE_TO_FPRIME_IMPORT = {'string': ['Fw/Types/EightyCharString.hpp'],
                             'time': ['Fw/Time/Time.hpp'],
                             'duration': ['Fw/Time/Time.hpp']}

def msg_field_to_fprime(pkg, field):
    """
    Converts a message type (e.g. uint32, std_msgs/String, etc.) into the FPrime type
    for that type (e.g. uint32_t, Fw::EightyCharString)

    @param type: The message type
    @type type: str
    @return: A tuple of the F Prime type, the size, and any imports necessary
    @rtype: str
    """
    fprime_type = None
    header_includes = []
    serial_imports = []
    array_len = None

    if field.is_header:
        field.base_type = 'std_msgs%sHeader'%SEP

    if (field.is_builtin):
        fprime_type = MSG_TYPE_TO_FPRIME[field.base_type]
        if field.base_type in MSG_TYPE_TO_FPRIME_IMPORT.keys():
            header_includes = MSG_TYPE_TO_FPRIME_IMPORT[field.base_type]
    else:
        fprime_type = _type_to_namespace(field.base_type)
        if SEP in field.base_type:
            splits = field.base_type.split(SEP)
            serial_imports = [_include_msg_serializable_path(splits[0], splits[1])]
        else:
            serial_imports = [_include_msg_serializable_path(pkg, field.base_type)]

    if field.is_array:
        if field.array_len is None or field.array_len > MAX_ARRAY_LEN:
            array_len = MAX_ARRAY_LEN
        else:
            array_len = field.array_len

    return (fprime_type, array_len, header_includes, serial_imports)
