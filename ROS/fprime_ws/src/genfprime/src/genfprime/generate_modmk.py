from __future__ import print_function

import os

from genmsg import MsgGenerationException

#from . name import *

## :param type_name outdir: Full path to output directory
## :returns int: status. 0 if successful
def write_modmk(outdir): #, msg_types, srv_types):
    if not os.path.isdir(outdir):
        #TODO: warn?
        return 0

    xml_in_dir = set([f for f in os.listdir(outdir)
                      if f.endswith('.xml')])
    _write_modmk(outdir, sorted(xml_in_dir))

# TODO(mereweth) if we want to independently specify the generated XML files
#     generated_xml = [_msg_serializable_xml_name(f) for f in sorted(msg_types)]
#     generated_xml.extend([_port_xml_name(f) for f in sorted(msg_types)]
#     write_msg_modmk(outdir, generated_xml)

#     generated_xml = [_srv_serializable_xml_name(f) for f in sorted(srv_types)]
#     generated_xml.extend([_port_xml_name(f) for f in sorted(srv_types)]
#     write_msg_modmk(outdir, generated_xml)

    return 0

def _write_modmk(outdir, generated_xml):
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    elif not os.path.isdir(outdir):
        raise MsgGenerationException("file preventing the creating of Fprime directory: %s"%dir)
    p = os.path.join(outdir, 'mod.mk')
    with open(p, 'w') as f:
        f.write('SRC = \\\n')
        for xml in generated_xml:
            f.write('xml\\\n'%xml)
    return 0
