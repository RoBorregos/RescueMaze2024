import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)  # Change '/dev/ttyUSB0' to the appropriate serial port on your system

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()  # Read a line from the serial port
            print(line)  # Print the received line
        
        # Example of sending data from Python to Arduino
            if line == "1":
                print("ENTRO NEGROS")
                ser.write(b'a')  # Send a string to Arduino


except KeyboardInterrupt:
    ser.close()  # Close the serial port when exiting the program