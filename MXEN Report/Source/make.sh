if avr-gcc -mmcu=atmega2560 -DF_CPU=16000000UL -DUSART_BAUDRATE=9600 -Os -g *.c; then
	if avr-objcopy -j .text -j .data -O ihex a.out main.hex; then
		if avrdude -patmega2560 -cwiring -P/dev/ttyACM0 -b115200 -D -Uflash:w:main.hex:i; then
			avr-size --format=avr --mcu=atmega32 a.out;
		fi
	fi
fi

rm a.out;
rm main.hex;
