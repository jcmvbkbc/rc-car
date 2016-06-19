CC = xtensa-lx106-elf-gcc
CFLAGS = -I. -mlongcalls
LDLIBS = -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -lc -Wl,--end-group -lgcc
LDFLAGS = -Teagle.app.v6.ld

rc-car-0x00000.bin: rc-car
	esptool.py elf2image $^

rc-car: rc-car.o

flash: rc-car-0x00000.bin
	esptool.py write_flash 0 rc-car-0x00000.bin 0x40000 rc-car-0x40000.bin

clean:
	rm -f rc-car rc-car.o rc-car-0x00000.bin rc-car-0x40000.bin
