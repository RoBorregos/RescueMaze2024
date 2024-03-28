import serial, time

arduino = serial.Serial('COM6', 9600)
time.sleep(2)
arduino.write(b'Hello World\n')
time.sleep(2)
msg = arduino.readline()
print(msg.decode('utf-8'))
arduino.close()
