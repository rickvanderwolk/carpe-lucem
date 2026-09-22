#!/usr/bin/env python3
"""Schrijft alles wat een ESP over USB stuurt weg naar een bestand, met tijdstempel.

Gebruik: python3 tools/serial-log.py /dev/cu.usbmodemXXXX logs/sensor.tsv

Opent de poort opnieuw als het bordje herstart, zodat een reset geen gat in de
log slaat. Zo kun je een dag meten zonder SD-kaart, zolang de Mac wakker blijft.
"""

import os
import select
import sys
import termios
import time
import tty

BAUD = termios.B115200


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = BAUD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    port, outpath = sys.argv[1], sys.argv[2]
    os.makedirs(os.path.dirname(outpath) or ".", exist_ok=True)

    fd = None
    buf = b""
    with open(outpath, "a", buffering=1) as out:
        while True:
            if fd is None:
                try:
                    fd = open_port(port)
                    out.write(f"# {time.strftime('%Y-%m-%d %H:%M:%S')}\tpoort open\n")
                except OSError:
                    time.sleep(1)
                    continue
            try:
                ready, _, _ = select.select([fd], [], [], 1)
                if not ready:
                    continue
                data = os.read(fd, 4096)
                if not data:
                    raise OSError("verbinding weg")
            except (OSError, BlockingIOError) as err:
                if isinstance(err, BlockingIOError):
                    continue
                out.write(f"# {time.strftime('%Y-%m-%d %H:%M:%S')}\tpoort weg\n")
                try:
                    os.close(fd)
                except OSError:
                    pass
                fd = None
                continue

            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if text:
                    out.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}\t{text}\n")


if __name__ == "__main__":
    main()
