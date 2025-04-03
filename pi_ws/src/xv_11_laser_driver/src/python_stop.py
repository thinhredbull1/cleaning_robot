#!/usr/bin/env python3

import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)
print(ser.name)

ser.write(b'$');
ser.write(b"stoplds$");
