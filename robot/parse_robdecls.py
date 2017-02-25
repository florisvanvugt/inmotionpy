

"""

What this will do is read robdecls.h and output a list of variables that we probably
want to be able to access through shared memory. For these variables, we write an on-the-fly C script
that will tell us the addresses of each item.

"""



objects_of_interest = [ "Ob",
                        "Robot",
                        "Daq",
                        "Game",
                        "Moh",
                        "Dyncmp_var" ]




import re


def comment_remover(text):
    """ 
    Shamelessly stolen from:
    http://stackoverflow.com/questions/241327/python-snippet-to-remove-c-and-c-comments 
    Thanks Internet and fellow programmers!
    """
    def replacer(match):
        s = match.group(0)
        if s.startswith('/'):
            return " " # note: a space and not an empty string
        else:
            return s
    pattern = re.compile(
        r'//.*?$|/\*.*?\*/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
        re.DOTALL | re.MULTILINE
    )
    return re.sub(pattern, replacer, text)    




def find_var_decls(line):
    """ Given a line in robdecls.h, find which variable(s) 
    of which type are defined.
    Returns a list where each element is a tuple (type,name,array_structure)
    representing single variable and its type, and array_structure is the 
    "array_structure", e.g. 8 if this is an 8-element array.

    TODO: Parse multidimensional arrays!
    """

    cont = line.strip()
    #m = re.match( r'^\s*(\w+)\s+((\w+)[\s,]+)+\s*$', cont) # doesn't work
    #m = re.match( r'^\s*(\w+)\s+([\w\s,]+)\s*$', cont)   # works but may become messy later on!

    firstsp = cont.find(' ')

    tp = cont[:firstsp]
    remain = cont[firstsp+1:]

    variables = []
    for v in remain.split(','):

        v = v.strip()
        
        # Is v a valid variable name?
        if re.match(r'\w+$', v):
            # Single variable
            variables.append((tp,v,None))
            continue

        # Try to parse it as an array
        m = re.match(r'^(\w+)\[(\d+)\]$',v)
        if m:
            grps = m.groups()
            variables.append((tp,grps[0],int(grps[1])))

        else:

            # Two-dimensional array! (Okay, this is bad form to be parsing it like this
            # but just for a quick-and-dirty solution!
            m = re.match(r'^(\w+)\[(\d+)\]\[(\d+)\]$',v)
            if m:
                grps = m.groups()
                totsize = int(grps[1])*int(grps[2])
                variables.append((tp,grps[0],totsize))

            else:

                print("// WARNING IGNORED variable name %s"%v)
            
    return variables



def parse_robdecls():
    """ 
    Now we need to know where C stores its stuff...
    We basically parse the typedefs in robdecls to find what the
    fields are called and what object they are found in.
    """

    contents = ""
    for fname in ['robdecls.h']:
        f = open(fname,'r')
        contents += f.read()
        f.close()

    # Strip away comments
    purec = comment_remover(contents)

    # Now look for typedef statements
    td = re.compile(r'typedef\s*struct\s*(\w+)\s*{([^}]*)}\s*(\w+);')
    matchs = td.findall(purec)
    structs = {}
    # For each typedef...
    for match in matchs:
        obname = match[-1]
        obcont = match[1]

        contents = [ itm.strip() for itm in obcont.split(";") ] # get the individual lines

        vrs = []
        for cont in contents:

            if len(cont)>0: # Silently ignore empty lines

                vrs += find_var_decls(cont)
                
        structs[obname]=vrs
        
    return structs






def expand_to_atomic(typename,typedefs):
    """ 
    Given a set of fields of a particular object, go through them one by one to
    see if they are atomic (e.g. f32 or such a "known" datatype). If they are not,
    i.e. if they are objects themselves, then go look for the appropriate object
    and "expand" this variable, e.g. given a variable pos of type xy in the typedef,
    and if the typdef of type xy consists of x and y, then return pos.x and pos.y, basically.

    We return a list of tuples (type,name,cname,array_type)
    where type is the data type (atomic), name is the variable name as we will
    be calling it in shared memory, cname is the name of the variable in c (i.e.
    with dots to "go into" objects, and array_type is whether it is an array (None if it isn't), and if so
    how many entries it has.
    """

    fields = [] # this is where we will stick the expanded variables
    
    for (tp,name,arystr) in typedefs[typename]:

        # If this is atomic
        if tp not in typedefs:
            fields.append( (tp,name,name,arystr) )

        else:

            if arystr==None: # We only expand stuff that is not an array
                
                # Let's expand it!
                subfields = expand_to_atomic(tp,typedefs)
                
                for (sub_tp,sub_name,sub_cname,sub_arystr) in subfields:
                    fields.append( (sub_tp,"%s_%s"%(name,sub_name),"%s.%s"%(name,sub_cname),sub_arystr) )


            else:
                print("\t// IGNORED %s,%s because it is an array of a non-atomic type (for type %s)"%(tp,name,typename))
                
                    
    return fields






def output_c_file(typdefs):

    print ("")
    print ("#include <stdlib.h>\n#include <signal.h>\n#include \"ruser.h\"\n#include \"rtl_inc.h\"\n#include \"robdecls.h\"\n\n")

    for typedef in objects_of_interest:
        print ("%s *%s;\n"%(typedef,typedef.lower()))

    print("\n\ns32 main(void) {\n")

    for typedef in objects_of_interest:

        thisobj = expand_to_atomic(typedef,typedefs)
    
        varname = typedef.lower()
        for (tp,name,cname,arraystruct) in thisobj:

            arrn = 0 if arraystruct==None else arraystruct
            print('\tprintf("%s %s %s %i %%u\\n", (unsigned int)&%s->%s ); // atomic'%(typedef,tp,name,arrn,varname,cname))


    print("\treturn 0;\n}\n\n")

    #res["xy"]





print ("// ##############################################################\n")
print ("// DO NOT EDIT -- TEMPORARY C File generated by parse_robdecls.py\n")
print ("// ##############################################################\n")

    
typedefs = parse_robdecls()


output_c_file(typedefs)
