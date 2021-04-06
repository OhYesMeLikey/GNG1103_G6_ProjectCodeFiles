import serial
from datetime import datetime
if __name__ == '__main__':
    flag = 0
    current_time = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
    file = open ( str(current_time)+".csv", 'w' )
    ser = serial.Serial('/dev/ttyS0', 19200, timeout=1)
    ser.flush()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            file.write (str(line))
    file.close()
