path=`pwd`

script=gen-findad.c

python3 parse_robdecls.py > ../robot/$script

cd ../robot

rm -f probe_addresses

gcc -Wall -I. -I/usr/realtime/include -O2 -I/lib/modules/2.6.15-interactive-motion.1500/build/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__ -march=i686 -Wall -pipe -fstrict-aliasing -Wno-strict-aliasing -I/home/floris/simple.robot/robot/../powerdaq/include -I/home/floris/simple.robot/robot/../pci4e -I/opt/imt/robot/powerdaq/include -I/opt/imt/robot/pci4e -g -O0 -o probe_addresses $script

# (This is mimicking $(CC) -Wall $(CFLAGS) -o findad shm-findad.c  -o findad shm-findad.c

cd $path

echo "#### Running it now ####"
../robot/probe_addresses > field_addresses.txt




