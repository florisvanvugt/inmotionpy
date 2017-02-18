

"""

What this will do is read robdecls.h and output a list of variables that we probably
want to be able to access through shared memory. For these variables, we write an on-the-fly C script
that will tell us the addresses of each item.

"""



objects_of_interest = [ "Ob",
                        "Robot",
                        "Daq",
                        "Game",
                        #"Moh",
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
    f = open('robdecls.h','r')
    robdecls = f.read()
    f.close()

    # Strip away comments
    purec = comment_remover(robdecls)

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










def output_c_file():

    print ("")
    print ("""
    #include <stdlib.h>
    #include <signal.h>
    #include "ruser.h"
    #include "rtl_inc.h"
    #include "userdecls.h"
    #include "robdecls.h"\n\n
    """)

    for typedef in objects_of_interest:
        print ("%s *%s;\n"%(typedef,typedef.lower()))

    print("\n\ns32 main(void) {\n")

    for typedef in objects_of_interest:

        varname = typedef.lower()
        for (tp,name,arraystruct) in typedefs[typedef]:

            if tp not in typedefs: # This is an "atomic" type; we don't have any other definition of it

                # Write the C code that will print the memory location of this variable (relative to
                # the start in memory of the main object, which is exactly what we want).
                arrn = 0 if arraystruct==None else arraystruct
                print('\tprintf("%s %s %s %i %%u\\n", (unsigned int)&%s->%s ); // atomic'%(typedef,tp,name,arrn,varname,name))

            else:
                # Actually this variable is of a type that we have a definition of.
                # Now it gets a bit trickier.
                # For instance, let's say that we have a variable here "pos" of type "xy" that elsewhere
                # is defined to have fields "x" and "y", which have atomic types (e.g. "f64").
                # Then here we will write memory locations for "pos.x" and "pos.y".

                if arraystruct==None:
                    for (subtp,subname,subarraystruct) in typedefs[tp]:
                        arrn = 0 if subarraystruct==None else subarraystruct
                        print('\tprintf("%s %s %s.%s %i %%u\\n", (unsigned int)&%s->%s.%s );'%(typedef,subtp,name,subname,arrn,varname,name,subname))
                else:
                    print("\t// IGNORED %s, %s because it is an array of a non-atomic type"%(tp,name))

    print("\treturn 0;\n}\n\n")

    #res["xy"]





print ("// ##############################################################\n")
print ("// DO NOT EDIT -- TEMPORARY C File generated by parse_robdecls.py\n")
print ("// ##############################################################\n")

    
typedefs = parse_robdecls()


output_c_file()
