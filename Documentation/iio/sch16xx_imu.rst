=========================
Murata SCH16xx IMU driver
=========================

1. Overview
===========
The SCH1600 is a combined high-performance 3-axis angular rate and 3-axis
accelerometer. The angular rate and accelerometer sensor elements are based on
Murata's proven capacitive 3D-MEMS technology. Signal processing is done by a
single mixed-signal ASIC that provides angular rate via a flexible SafeSPI
v2.0 compliant digital interface. Sensor elements and ASIC are packaged to
pre-molded SOIC 24-pin plastic housing that guarantees reliable operation
over the product's lifetime.

The SCH1600 is designed, manufactured, and tested for high stability,
reliability, and quality requirements. The component has extremely stable
output over temperature, humidity, and vibration. It is qualified according to
the AEC-Q100 standard and has several advanced self-diagnostic features. The
component is suitable for SMD mounting and is compatible with RoHS and ELV
directives

2. Driver details
=================
The device can be instantiated via device tree bindings. Dynamic range of the
sensors can be selected with vendor specific device tree properties.

SCH16xx IIO driver implements channels IIO_ANGL_VEL, IIO_ACCEL, IIO_TEMP and
IIO_TIMESTAMP in triggered buffer mode. Typically the sensor readout is
triggered with a hrtimer trigger.

Additionally the driver exposes some vendor specific attributes:

serial_number
	Serial number of the sensor read from traceability registers SN_ID1,
	SN_ID2 and SN_ID3.

product_code
	Product code ie. component version read from COMP_ID register,
	eg. SCH1633-B01.

anglvel_range
	Measurement range of the gyroscope using the current dynamic range
	settings in deg/s.

accel_range
	Measurement range of the accelerometer using the current dynamic range
	settings in m/s^2.
