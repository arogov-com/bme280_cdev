obj-m += bme280.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

load:
	sudo insmod ./bme280.ko

unload:
	sudo rmmod ./bme280.ko
