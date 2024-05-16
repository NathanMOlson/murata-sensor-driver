# murata-sensor-driver

Linux kernel driver for murata SCH16XX sensors.

## Compiling and testing the driver

Makefile is supplied in `drivers/iio/imu/murata` to compile the driver out of
kernel tree using compiled kernel headers in `/usr/src/linux-headers-<kernel version>`.
Running
```
	make
```
should build the driver module `sch16xx.ko`.

Define and load a device tree block to instantiate a sensor. There is a device tree
overlay for Raspberry pi in `drivers/iio/imu/murata/testing/sch16xx_overlay.dts`.

The script `/drivers/iio/imu/murata/testing/test_sch16xx.sh` compiles and loads the device tree
overlay, and loads the necessary modules. After that,
```
	make
	insmod sch16xx.ko
```
will load the driver, and a device in `/sys/bus/iio/devices/` should be instantiated.
