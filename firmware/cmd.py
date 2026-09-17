#!/usr/bin/env python3
"""Send one console command to a board and print what comes back.

usage: cmd.py /dev/ttyACMx "command" [seconds]
"""
import os, select, sys, termios, time

port, command = sys.argv[1], sys.argv[2]
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
fd = os.open(port, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
a = termios.tcgetattr(fd)
a[0] = a[1] = a[3] = 0
a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
a[4] = a[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, a)
# drain anything pending
while select.select([fd], [], [], 0.05)[0]:
    try:
        os.read(fd, 4096)
    except BlockingIOError:
        break
# a bare CR first: flushes whatever the board's line buffer holds
for _ in range(60):
    try:
        os.write(fd, b"\r")
        break
    except BlockingIOError:
        time.sleep(0.05)
time.sleep(0.15)
while select.select([fd], [], [], 0.05)[0]:
    try:
        os.read(fd, 4096)
    except BlockingIOError:
        break
os.write(fd, command.encode() + b"\r")
t0 = time.time()
out = b""
while time.time() - t0 < secs:
    if select.select([fd], [], [], 0.2)[0]:
        try:
            out += os.read(fd, 4096)
        except BlockingIOError:
            pass
text = out.decode(errors="replace")
# hide the periodic heartbeat unless asked for
if "--all" not in sys.argv:
    text = "\n".join(l for l in text.splitlines() if not l.startswith(("sensor-board (built", "clocks:")))
print(text.strip())
