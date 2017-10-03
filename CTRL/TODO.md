## enable deployment without autocoded topology - minimal working example
1. C modules
    1. RateGroup driver
    1. Passive RateGroup
1. PRU modules
    1. PRU to ARM signalling
    1. PWM
    1. RPmsg?

## autocoded C with function pointers connected by autocoder
codegen.py line 232 selects which visitor for a file

is there an equivalent of serializables?

autobuild function declaration/definition and argument list / return value? (equivalent of ports)

need to build components?

line 1084, add new xml_type for C "assembly" - fprime word for connected components
     if xml_type == "Cdeployment":

generate topology, which autoconnects function pointers