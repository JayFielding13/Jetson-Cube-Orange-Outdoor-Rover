import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)
print('Arduino data stream:')

for i in range(10):
    data = ser.readline().decode('utf-8', errors='ignore').strip()
    if data:
        print(f'{i+1}: {data}')

ser.close()
