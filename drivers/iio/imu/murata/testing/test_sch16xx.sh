echo "Compiling the device tree overlay..."
dtc -O dtb -o sch16xx_overlay.dtbo sch16xx_overlay.dts

echo "Inserting the device tree overlay..."
dtoverlay sch16xx_overlay.dtbo

echo "Loading the required modules..."
modprobe industrialio
modprobe industrialio-triggered-buffer
modprobe iio-trig-hrtimer
modprobe crc8
