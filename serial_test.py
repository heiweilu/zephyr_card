import serial, time
try:
    s = serial.Serial('COM7', 115200, timeout=2)
    s.reset_input_buffer()
    time.sleep(0.5)
    s.write(b'tune get\r\n')
    time.sleep(2)
    n = s.in_waiting
    print(f'Bytes waiting: {n}')
    if n > 0:
        data = s.read(n)
        print('Response:')
        print(data.decode('utf-8', 'replace'))
    else:
        print('No data received from ESP32!')
    s.close()
    print('Serial closed OK')
except Exception as e:
    print(f'ERROR: {e}')
