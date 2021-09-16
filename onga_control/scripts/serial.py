import serial

ser = serial.Serial('COM3',115200)  # open serial port
# ser.port = 'COM3'
# ser.setbaudrate = 115200
print(ser.name)         # check which port was really used
# ser.open()
# ser.write(b'0.5,0,0.0\n\n')     # write a string
ser.write(b'0.0,0,0.0\n\n')     # write a string
while(1) :
    pass
# ser.close()             # close port