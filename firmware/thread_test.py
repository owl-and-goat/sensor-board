#!/usr/bin/env python3
"""Two-board Thread test over the USB consoles.

usage: thread_test.py /dev/ttyACM_A /dev/ttyACM_B

Board A forms a network (leader) and B joins it with A's active dataset;
then each pings the other's mesh-local address and both print their neighbor
tables (RSSI). Assumes both boards run the sensor-board firmware with the
Thread FTD stack installed and started on CPU2 (see README.org).
"""
import os, re, select, sys, termios, time


def write_retry(fd, data, secs=3.0):
    """Non-blocking write with retry: right after enumeration the tty can
    report EAGAIN for a moment."""
    t0 = time.time()
    while True:
        try:
            os.write(fd, data)
            return
        except BlockingIOError:
            if time.time() - t0 > secs:
                raise
            time.sleep(0.05)


class Console:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        self.name = port
        write_retry(self.fd, b"\r")
        time.sleep(0.2)
        self.drain()

    def drain(self):
        while select.select([self.fd], [], [], 0.05)[0]:
            try:
                os.read(self.fd, 4096)
            except BlockingIOError:
                break

    def cmd(self, line, secs=3.0, quiet=False):
        self.drain()
        write_retry(self.fd, line.encode() + b"\r")
        t0, out = time.time(), b""
        while time.time() - t0 < secs:
            if select.select([self.fd], [], [], 0.2)[0]:
                try:
                    out += os.read(self.fd, 4096)
                except BlockingIOError:
                    pass
        lines = [l for l in out.decode(errors="replace").splitlines()
                 if l and not l.startswith(("sensor-board (built", "clocks:", "cpu2:")) and l.strip() != line]
        if not quiet:
            for l in lines:
                print(f"  [{self.name}] {l}")
        return "\n".join(lines)


def main():
    a, b = Console(sys.argv[1]), Console(sys.argv[2])
    print("== board A: form network ==")
    a.cmd("thread init", 3)
    a.cmd("ot init", 2)
    a.cmd("ot down", 2, quiet=True)
    a.cmd("ot new", 3)
    a.cmd("ot up", 3)
    print("== board B: init ==")
    b.cmd("thread init", 3)
    b.cmd("ot init", 2)
    b.cmd("ot down", 2, quiet=True)
    print("== wait for A to become leader ==")
    for _ in range(20):
        info = a.cmd("ot info", 2, quiet=True)
        if "role leader" in info:
            break
        time.sleep(1)
    print(f"  [A] {info}")
    raw = a.cmd("ot tlvs", 3, quiet=True)
    n = re.search(r"active tlvs -> ok \((\d+) bytes\)", raw)
    m = re.search(r"ottlvs ([0-9a-f]+)", raw)
    if not (n and m):
        sys.exit(f"no active dataset from A:\n{raw}")
    tlvs = m.group(1)
    if len(tlvs) != 2 * int(n.group(1)):
        sys.exit(f"dataset hex truncated ({len(tlvs)//2} of {n.group(1)} bytes):\n{raw}")
    print(f"  [A] dataset {n.group(1)} bytes")
    print("== board B: join with A's dataset ==")
    b.cmd(f"ot settlvs {tlvs}", 3)
    b.cmd("ot up", 3)
    for _ in range(40):
        info = b.cmd("ot info", 2, quiet=True)
        if "role child" in info or "role router" in info:
            break
        time.sleep(1)
    print(f"  [B] {info}")
    ma = re.search(r"mleid ([0-9a-f:]+)", a.cmd("ot info", 2, quiet=True))
    mb = re.search(r"mleid ([0-9a-f:]+)", info)
    if not (ma and mb):
        sys.exit("missing mesh-local addresses")
    print("== ping A -> B ==")
    a.cmd(f"ot ping {mb.group(1)} 5", 8)
    print("== ping B -> A ==")
    b.cmd(f"ot ping {ma.group(1)} 5", 8)
    print("== neighbor tables (RSSI) ==")
    a.cmd("ot neighbors", 3)
    b.cmd("ot neighbors", 3)


if __name__ == "__main__":
    main()
