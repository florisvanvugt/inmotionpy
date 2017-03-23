# PowerDAQ dummy

## What is this
This is a dummy version of the PowerDAQ driver. It allows you to load a pwrdaq module into the kernel so that you can run the robot, even if no PowerDAQ board is physically present.

## How to use

Compile the code:

``` 
make
```

Then 

```
sudo insmod pwrdaq_dummy.ko
```

This is known to work with Xenomai 2.6 and Vanilla Linux kernel 3.18.20 on Ubuntu 12.04/x86.


