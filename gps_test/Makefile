	PWD := $(shell pwd)
target:
	#arm-fsl-linux-gnueabi-gcc -L$(PWD)/nmea -lnmeagps -lm  GPS_NEO_6M.c -o GPS_NEO_6M
	arm-linux-gcc -L$(PWD)/nmea -lnmeagps -lm  GPS_NEO_6M.c -o GPS_NEO_6M
clean:
	rm -f GPS_NEO_6M
