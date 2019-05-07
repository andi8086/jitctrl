MCU = Atmega328P
ARDUINOTTY ?= /dev/ttyUSB0
IFACE = arduino

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

main.elf: main.o
	avr-ld -mavr5 -o $@ $<

main.o: main.s
	avr-as -a -mmcu=$(MCU) -o $@ $< > $<.lst

prog: main.hex
	avrdude -p $(MCU) -F -e -c $(IFACE) -P $(ARDUINOTTY) -Uflash:w:"$<":i

sim: main.hex
	simavr -f 16000000 -m atmega328p -pty0 $<
