import serial
import time

ser = serial.Serial('/dev/tty.usbserial-02B91ADF', 115200)

def send_command(command):
    if command[-1] != '\n':
        command += '\n'
    ser.write(command.encode())
    time.sleep(0.1)
    response = ser.read(ser.inWaiting())
    return response

# send_command('CHNL 0; Tick 0.85 20;\n')
# time.sleep(1)
send_command('CHNL 1; vibrate 100 0.3 1000 1 1;')
# time.sleep(10)
send_command('CHNL 0;F 4000 20;PCM 244,0,0,0,0,0,0,0,0,0,0,255,0,0,0,045,45,45,45;\n')
time.sleep(4)

ser.close()
