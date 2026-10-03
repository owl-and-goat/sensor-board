#!/usr/bin/env python3
"""Minimal console for the sensor board: /dev/ttyACM0 <-> your terminal.

Opens the port non-blocking and applies termios with TCSANOW, so it never
hangs the way `stty`/`cat` can when the port has undrained output. Type
`dfu` + Enter to reboot the board into the ROM bootloader. Ctrl-C to quit.
"""
import os, select, sys, termios, time, tty

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
fd = os.open(port, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
a = termios.tcgetattr(fd)
a[0] = a[1] = a[3] = 0
a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
a[4] = a[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, a)

stdin = sys.stdin.fileno()
interactive = os.isatty(stdin)
if interactive:
    saved = termios.tcgetattr(stdin)
    tty.setcbreak(stdin)
try:
    while True:
        r, _, _ = select.select([fd, stdin], [], [], 1.0)
        if fd in r:
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                data = b""
            if data:
                sys.stdout.write(data.decode(errors="replace"))
                sys.stdout.flush()
        if stdin in r:
            data = os.read(stdin, 1024)
            if not data:
                break
            os.write(fd, data.replace(b"\n", b"\r"))
except KeyboardInterrupt:
    pass
finally:
    if interactive:
        termios.tcsetattr(stdin, termios.TCSANOW, saved)
