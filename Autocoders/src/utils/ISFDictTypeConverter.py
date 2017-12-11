
class ISFDictTypeConverter(object):
    
    def __init__(self):
        pass
    
    def convert(self, t,size):

        # check for various type variations
        type_string = ""
        type_name = t
        ser_import = None
        # check for enums
        if (type(t) == type(tuple())):
            # extract enumeration arguments
            # to match the C rules, we have to start 
            # counting member values from 0 or the 
            # last member value
            curr_memb_val = 0
            # make sure it's an enum
            if t[0][0].upper() != "ENUM":
                print("ERROR: Expected ENUM type in channel args list...")
                sys.exit(-1)
            enum_type = t[0][1]
            type_string += "IsfEnumType(\"" + t[0][1] + "\",{"
            
            for (mname,mval,mcomment) in t[1]:
                # check for member value
                if mval != None:
                    curr_memb_val = int(mval)
                type_string += "\"%s\":%d,"%(mname,curr_memb_val)
                curr_memb_val += 1
            type_string += "})"
            type_name = "enum"
        # otherwise, lookup type translation in table
        elif t == "string":
            type_string += "IsfStringType(max_string_len=%s)"%size
        else:
            type_lookup = {
               "U8":"IsfU8Type()",
               "I8":"IsfI8Type()",
               "I16":"IsfI16Type()",
               "U16":"IsfU16Type()",
               "I32":"IsfI32Type()",
               "U32":"IsfU32Type()",
               "I64":"IsfI64Type()",
               "U64":"IsfU64Type()",
               "F32":"IsfF32Type()",
               "F64":"IsfF64Type()",
               "bool":"IsfBoolType()",
            }
            if type_lookup.has_key(t):
                type_string += type_lookup[t]
            else: # set up serializable imports
                # Path to serializable is going to be the namespace.type
                ser_type = t.split("::")
                type_string += "%s.%s()" %(".".join(ser_type),ser_type[-1])
                ser_import = ".".join(ser_type)
        return (type_string,ser_import,type_name)
    
    def format_replace(self, format_string, spec_num, old, new):
        """
        Search the format specifier string and replace tokens
        Mainly a special case to handle enumerations. Software 
        needs "%d", while Gse needs "%s"
        spec_num = instance of token (0..n)
        """
        # split into list
        flist = format_string.split('%')
        # make sure we're not looking past the list
        if spec_num + 1 >= len(flist):
            return None 
        # replace token
        flist[spec_num+1] = flist[spec_num+1].replace(old,new,1)
        # rejoin string
        return "%".join(flist)
        
                
    