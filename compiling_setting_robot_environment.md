
# Compiling Robot Code and Xenomai Kernel

written by Floris van Vugt, February 2017.

The instructions below are based on trial and lots of error and some very useful READMEs in various packages. There may be restarts required in between, or `sudo ldconfig` to be run in between parts of the instructions, but I am not sure because I did a lot of back and forth when I tried to get this working. So I hope it is possible more or less linearly as listed here below.


## In short

In short, you need to set up an environment which contains a custom compiled Linux 2.6 kernel, which needs to be patched to be able to work with Xenomai, which enables realtime functionality. Once the kernel is compiled and install, you compile and install Xenomai, and then you compile and install drivers for the interface cards, PowerDAQ. That is it. 



## Sources

You can find a source tree for the kernel and xenomai patches in `/opt/imt/`. In particular, I found a helpful file `imt-readme.txt` in `robot3.0.3beta010` and that is what most of this is based on. Also there was `imt-notes.txt` which listed some stuff that surprisingly wasn't mentioned in the readme, but which was important I think.




## Background

Some information about how people compiled kernels back then.

* [Thread](https://ubuntuforums.org/archive/index.php/t-24853.html)
* [Kernel compilation for newbies](https://ubuntuforums.org/showthread.php?t=56835&page=18)


## Current set up (Suzuki Feb 2017)

I think Suzuki is using a custom kernel 2.6.15 with Xenomai 2.0.3.

In order to use PowerDAQ we need to have 2.2 - 2.6 Linux Kernels.



## Prelims

You need a basically sane build environment to be able to compile kernel stuff.

```
sudo apt-get install build-essential kernel-package fakeroot
sudo apt-get install libncurses5-dev libncursesw5-dev # required for building xenomai
sudo touch /etc/ld.so.conf
```

I think it also good to tell the system which robot code we are using. The system seems to expect `/opt/imt/robot` to point to the latest robot code. This is used when compiling robot code to find the `powerdaq` sources, and perhaps at other occasions too.

```
sudo rm -f /opt/imt/robot # remove previous symbolic link
sudo ln -s <<YOURrobot-suiteDIRECTORY>> /opt/imt/robot # point to the new robot
``` 





## Building the kernel

```
cd <<YOURrobot-suiteDIRECTORY>>/linux-2.6.15/
```

This sets everything up:
```
make-kpkg clean
```

Note that when you run this, it fails because it can't find `arch/um/`. I wonder why. I just moved on.

```
You may need root privileges - some parts may fail
/usr/bin/make -f /usr/share/kernel-package/rules real_stamp_clean
make[1]: Entering directory `/home/floris/robot3.0.3beta010/linux-2.6.15'
test ! -f .config || cp -pf .config config.precious
test -f Makefile && \
            /usr/bin/make    ARCH=i386 distclean
make[2]: Entering directory `/home/floris/robot3.0.3beta010/linux-2.6.15'
/home/floris/robot3.0.3beta010/linux-2.6.15/fs/hostfs/Makefile:11: arch/um/scripts/Makefile.rules: No such file or directory
make[4]: *** No rule to make target `arch/um/scripts/Makefile.rules'.  Stop.
make[3]: *** [fs/hostfs] Error 2
make[2]: *** [_clean_fs] Error 2
make[2]: Leaving directory `/home/floris/robot3.0.3beta010/linux-2.6.15'
make[1]: [real_stamp_clean] Error 2 (ignored)
test ! -f config.precious || mv -f config.precious .config
test ! -f stamp-patch || /usr/bin/make -f /usr/share/kernel-package/rules unpatch_now
test -f stamp-building || test -f debian/official || rm -rf debian
# work around idiocy in recent kernel versions
test ! -e scripts/package/builddeb.dist || \
            mv -f scripts/package/builddeb.dist scripts/package/builddeb
test ! -e scripts/package/Makefile.dist || \
            mv -f scripts/package/Makefile.dist scripts/package/Makefile
rm -f modules/modversions.h modules/ksyms.ver debian/files conf.vars scripts/cramfs/cramfsck scripts/cramfs/mkcramfs applied_patches debian/buildinfo stamp-build stamp-configure stamp-source stamp-image stamp-headers stamp-src stamp-diff stamp-doc stamp-buildpackage stamp-libc-kheaders stamp-debian stamp-patch stamp-kernel-configure
rm -rf debian/tmp-source debian/tmp-headers debian/tmp-image debian/tmp-doc
make[1]: Leaving directory `/home/floris/robot3.0.3beta010/linux-2.6.15'
```

It can be helpful to look at the kernel-compiling documentation (`man make-kpkg`).


Actual kernel compilation; will take about 30 minutes on Suzuki.
```
make-kpkg --initrd -rootcmd fakeroot --append-to-version -custom.<<VERSION>> kernel_image modules_image
```

It will complain that sources already exist (here I used `-fvv001` as version string).

```
(...)
Warning: The file include/linux/version.h exists
The contained UTS_VERSION string:
                        "2.6.15-interactive-motion.1500"
does not match expectations:
                        "2.6.15-fvv001"
I'll try and recover
# work around idiocy in recent kernel versions
(...)
```


Installing the kernel image.
```
sudo dpkg -i ../kernel-image-2.6.15-custom.747_10.00.Custom_i386.deb
```

At this point in the `readme.txt` file there is talk of a second `.deb` file that we are supposed to install with `dpkg -i`, but it seems to have the same name...?

Now the next lines apparently adjust the initial RAM disk for loading the kernel. In the `dpkg` command a ramdisk was created, but this somehow replaces it, I believe.

```
sudo mkdir /lib/modules/2.6.15-custom.<<VERSION>>/boot
sudo cp -a /lib/modules/2.6.15-custom.<<VERSION>>/kernel/security/capability.ko /lib/modules/2.6.15-custom.<<VERSION>>/boot/
sudo mkinitramfs -o /boot/initrd.img-2.6.10-custom.<<VERSION>> 2.6.10-custom.<<VERSION>>
```

Note: `mkinitrd` was not found, and according to some links this has been superceded by `mkinitramfs`. I am not sure whether the syntax is the same though, so I hope it is.

Next, the original instructions said we should edit `/boot/grub/menu.lst` and add an `initrd` line after the `kernel` line corresponding to the kernel we just built. However, when I ran this the `initrd` line seemed to already be there.





## Xenomai


Xenomai is what makes your kernel real-time. In actual fact I think it is a kind of second kernel that takes care of all the really important stuff directly and then passes the rest on to the "normal" kernel. Xenomai is based on Adeos which is something that has to be added to the kernel I think.

You can download Xenomai from [here](https://git.xenomai.org/). I tried both 2.0.3 and 2.0.4. You can find them separately [here](https://xenomai.org/downloads/xenomai/stable/).
I went for version 2.0 (i.e. 2.0.4).


First, make sure the Linux kernel source that we actually used is behind `/usr/src/linux` because that is where the xenomai sources will expect it.

```
(not needed?-> # 
sudo ln -s <<YOURrobot-suiteDIRECTORY>>/linux-2.6.15/ /usr/src/linux
sudo ln -s <<YOURrobot-suiteDIRECTORY>>/linux-2.6.15/ /lib/modules/`uname -r`/build
```

Build Xenomai. 

```
cd xenomai  # or wherever your sources of Xenomai are
mkdir build
cd build
make -f ../makefile menuconfig
```

This will launch a menu in which I think you can just use the default options.

```
make
```

The line below will install Xenomai to `/usr/realtime` by default, I believe.

```
sudo make install
```


I could not find any existing `.xeno_config` on Suzuki so we had to go from scratch.


Also, it may be good to point the shared library path to `/usr/realtime/lib` where libraries were installed, i.e. add it to `/etc/ld.so.conf`.
```
# Run this as super user
echo /usr/realtime/lib >> /etc/ld.so.conf
```


## PowerDAQ 

PCI card to control the robot I believe. Here is a [manual](https://www.omega.com/manuals/manualpdf/M3831.pdf) I found. I couldn't find an original copy of the drivers we used anywhere on the net, but there is [this](http://www.ueidaq.com/cms/publications/interfacing-da-hardware-to-linux/) (which is not what I used in the description below).

```
cd <<YOURrobot-suiteDIRECTORY>>/powerdaq
```

PowerDAQ needs to know where to find the Xenomai files, so we open `Makefile` and set `XENOMAI_DIR` to the correct location.

Edit the PowerDAQ Makefile and set the variable `XENOMAI_DIR` to the location
of your Xenomai installation (`/usr/realtime` in our examples I believe).

I like to run this, but there is also a script `./cleanup` that you can run.
```
make clean
```


```
make XENOMAI=1
sudo make install
```

During `make` I get a few warnings:
```
*** Warning: "rtdm_dev_register" [/home/floris/robot3.0.3beta010/powerdaq/pwrdaq.ko] undefined!
*** Warning: "rtdm_dev_unregister" [/home/floris/robot3.0.3beta010/powerdaq/pwrdaq.ko] undefined!
*** Warning: "rthal_domain" [/home/floris/robot3.0.3beta010/powerdaq/pwrdaq.ko] undefined!
```

It says that:
```
PowerDAQ driver for Linux is now installed
You can load the driver with the command 'modprobe pwrdaq'
You can compile the examples with the command 'make test'
```

Note that you cannot just modprobe the driver though, because it depends on Xenomai to also be loaded previously.





## PCI4e

**NOTE** PCI4e is not actually required for the robot anymore. It is probably an interface card that was being used at some point but no longer is. For one, I could not find it physically in suzuki. Nevertheless, here below I will write what I could figure out about the code to interface with this card.


The PCI-4E is again an interface card, see its [manual](usdigital.com/assets/datasheets/PCI-4E_datasheet.pdf?k=635168184226049538). You can download drivers [here](http://wa.us.usdigital.com/support/software/pci-4e).

Note that PCI4e also requires Linux 2.6 kernel.

This section is based largely on `pci4e/PCI4E_README`.

If you haven't done so yet, link `/usr/src/linux` to the correct kernel source tree.
```
ln -s <<YOURrobot-suiteDIRECTORY>> /usr/src/linux
```

Compile the PCI4e code:

```
cd <<YOURrobot-suiteDIRECTORY>>/pci4e
make clean
make 
```

Mark as executable the items you are going to need:

```
chmod +x load
chmod +x pci4e_*
chmod +x pci4edemo
```



## Run robot code

Some checks for how things should be set up now:

* `/lib/modules/<YOURKERNEL>/kernel/drivers/misc/pwrdaq.ko` should exist, this is the PowerDAQ compiled code (kernel module).
* There should be various files `/usr/realtime/modules/xeno_XXX.ko` (Xenomai kernel modules)
* `/opt/imt/robot` should contain, or be a symbolic link to, the robot source tree directory, i.e. it should have among other things a subdirectory `pci4e` where the `pci4e_load` executable is located.


Some pointers:

* Make sure you recompile the robot sources against the new kernel!

* You might also want to change `checkexist` in the most current robot source tree, since it tends to check whether your kernel module contains `interactive-motion` and if not, refuse to run.



## Diagnostics

A few things seem to not work or give error messages and I don't understand why.

1. When you load `pci4e`, it complains on `dmesg` that it can't find the card with specs `1892:5747` (vendor and device ID, I believe). I confirmed that this very same error message appears on the old (working) Suzuki installation as well. No module named `pci4e` appears to be ever loaded. How can it be that everything works correctly when a seemingly important element of the system is missing?

2. PowerDAQ loading often complains. This was solved when I compiled it against the correct Xenomai sources.

Here is some example `dmesg` output when the robot code is launched. This is on my new installation, not on the "old" Suzuki installation:

```
[  896.834048] I-pipe: Domain Xenomai registered.
[  896.834056] Xenomai: hal/x86 loaded.
[  896.969801] Xenomai: real-time nucleus v2.0.3 (Hill Groove) loaded.
[  897.154751] Xenomai: starting native API services.
[  897.184249] Xenomai: starting RTDM services.
[  897.214536] pd: PowerDAQ Driver 3.6.11, Copyright (C) 2000,2005 United Electronic Industries, Inc.
[  897.214561] PCI: Found IRQ 10 for device 0000:07:00.0
[  897.234738] pd: Board 0:
[  897.234741] pd:      Name: PD2-MF-16-50/16L
[  897.234742] pd:      Serial Number:  0016219
[  897.234744] pd:      Input FIFO size: 1024 samples
[  897.234746] pd:      Input channel list FIFO size: 1024 entries
[  897.234748] pd:      Output FIFO size: 2048 samples
[  897.234750] pd:      Manufacture date: 01-MAY-2001
[  897.234752] pd:      Calibration date: 21-JUN-2001
[  897.234754] pd:      Base address: 0x32010000
[  897.234756] pd:      IRQ line: 0xa
[  897.234757] pd:      DSP Rev: v2
[  897.234759] pd:      Firmware type: MFx, rev: 3.45/50113
[  897.234775] PCI: Found IRQ 9 for device 0000:07:02.0
[  897.234788] PCI: Sharing IRQ 9 with 0000:00:1c.2
[  897.234808] PCI: Sharing IRQ 9 with 0000:00:1d.2
[  897.234823] PCI: Sharing IRQ 9 with 0000:00:1f.1
[  897.254916] pd: Board 1:
[  897.254918] pd:      Name: PD2-MF-16-50/16H
[  897.254920] pd:      Serial Number:  0013005
[  897.254922] pd:      Input FIFO size: 1024 samples
[  897.254923] pd:      Input channel list FIFO size: 256 entries
[  897.254925] pd:      Output FIFO size: 2048 samples
[  897.254927] pd:      Manufacture date: 01-JUN-2000
[  897.254929] pd:      Calibration date: 13-OCT-2000
[  897.254931] pd:      Base address: 0x32000000
[  897.254932] pd:      IRQ line: 0x9
[  897.254934] pd:      DSP Rev: v2
[  897.254936] pd:      Firmware type: MFx, rev: 3.45/50113
```



## Troubleshooting

### PowerDAQ
I was puzzled for a while about Powerdaq.

During compiling it already gave some warnings that it couldn't find symbols.

Then when I try to load the module (modprobe) then it fails saying 
```
FATAL: Error inserting pwrdaq (/lib/modules/2.6.15-fvv001/kernel/drivers/misc/pwrdaq.ko): Unknown symbol in module, or unknown parameter (see dmesg)
```

And in `dmesg` it says:
```
[17180184.092000] pwrdaq: Unknown symbol rthal_domain
[17180184.092000] pwrdaq: Unknown symbol rtdm_dev_unregister
[17180184.092000] pwrdaq: Unknown symbol rtdm_dev_register
```
Incidentally, if you go with `make` simply (without Xenomai) then it *does* compile and works with `make install` and even with the robot itself. However, I think it may defy the purpose because then it doesn't run in realtime?

**The solution** is simply to first load the xenomai module on which this guy depends. If you first load `xeno_rtdm` module, then you can safely load `pwrdaq`.




## Upgrading to a new Linux kernel

Igo Krebs told me that this is the setup they currently use:

* Ubuntu 14.04 (14.04 means 2014 April). We are actually using the Xubuntu variant of this, which is similar to regular Ubuntu, but a bit "leaner."
* We make our own Linux kernel, which is version 3.14.17
* This is running with Xenomai version 2.6.4 (which runs a patch on our kernel).

https://git.xenomai.org/


## General notes about the Suzuki computer

### Video card
The video card is ASUS GeForce 7300GT DirectX 9 EN7300GT SILENT/HTD/256M 256MB 128-Bit GDDR2 PCI Express x16 SLI Support Video Card.

The video card doesn't work with the X windowing system out of the box on Ubuntu 5.04 but it *does* work out of the box on Ubuntu 12.04. To get it to work under 5.04 this is what I did. I downloaded [these NVIDIA drivers](http://www.nvidia.com/download/driverResults.aspx/73965/en-us).

I executed the shell script. First it failed because it couldn't find kernel headers, so I installed linux-headers-XX where XX was determined from `uname -r`. Then it complained that the current C compiler (GCC 4.0) was different than the one used to compile the running kernel (3.4). Forcing it to go ahead didn't work: the kernel module wouldn't load. 
Then I installed gcc-3.4 and set the environment variable `export CC=/usr/bin/gcc-3.4` and then ran the NVIDIA script again.
This time it compiled, and seemed to be okay with the kernel module, and then it offered me to custom make the X11 config script, which I did, and with that it ended up with a working X11 graphics.

To get tcl::snack working, I compiled it with OSS support (not ALSA, as it will offer). This caused slightly better performance, I believe.


### PCI data acquisition/control cards

The robot interface is through two PowerDAQ cards, by United Electronic Industries, models PD2-MF-16-50/16L and PD2-MF-16-50/16H (these cards are so old you will probably not find data sheets for these specific cards anymore.


