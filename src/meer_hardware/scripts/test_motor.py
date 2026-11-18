#!/usr/bin/env python3
"""Drive the Motor ESP directly (no ROS) to prove the Pi -> ESP link works.

WHEELS OFF THE GROUND FIRST!

  USB cable (default):  python3 test_motor.py serial /dev/rover_motor 0 30 2
  WiFi UDP:             python3 test_motor.py udp 192.168.1.82 0 30 2

Arguments: <serial|udp> <port-or-ip> <x turn> <y drive> <seconds>
Sends "x,y" at 10 Hz for <seconds>, then "0,0".
"""
import socket
import sys
import time

mode = sys.argv[1] if len(sys.argv) > 1 else 'serial'
target = sys.argv[2] if len(sys.argv) > 2 else ('/dev/rover_motor' if mode == 'serial' else '192.168.1.82')
x = int(sys.argv[3]) if len(sys.argv) > 3 else 0
y = int(sys.argv[4]) if len(sys.argv) > 4 else 30
dur = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0

if mode == 'serial':
    import serial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = target, 115200, 0.1
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(0.5)

    def send(msg):
        ser.write((msg + '\n').encode())
else:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(msg):
        sock.sendto(msg.encode(), (target, 4210))

end = time.time() + dur
while time.time() < end:
    send(f'{x},{y}')
    time.sleep(0.1)
for _ in range(5):
    send('0,0')
    time.sleep(0.05)

if mode == 'serial':
    reply = ser.read(ser.in_waiting or 1).decode(errors='ignore').strip()
    if reply:
        print('ESP said:', reply[-200:])
print(f'Sent "{x},{y}" for {dur}s then "0,0" via {mode} -> {target}')
