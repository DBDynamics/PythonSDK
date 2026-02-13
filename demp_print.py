import serial
import time

try:
    ser = serial.Serial(
        port='/dev/ttyS9',
        baudrate=115200,
        timeout=1
    )
    ser.write("hello world\n".encode('utf-8'))
    time.sleep(0.5)
    response = ser.read(100)
    if response:
        print(response.hex())
except Exception:
    pass
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()