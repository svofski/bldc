#avrdude -p m8 -c avrispmkii -U lfuse:w:0xef:m -U hfuse:w:0xdc:m 
avrdude -p m8 -c avrispmkii -U flash:w:"boot_atmega8.hex":i -v -V -u
