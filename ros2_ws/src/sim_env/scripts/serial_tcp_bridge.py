#!/usr/bin/env python3
"""Relay a serial port to a TCP server, reconnecting on either side. Use it where the serial
device is not visible to the simulator host, e.g. Windows COM port -> WSL2:

    python serial_tcp_bridge.py COM10 --baud 921600 --host localhost --port 8888

Needs pyserial. DTR/RTS stay low so opening the port does not reset an ESP32.
"""
import argparse
import socket
import threading
import time

import serial


def pump(src_read, dst_write, stop):
    try:
        while not stop.is_set():
            data = src_read()
            if data:
                dst_write(data)
    except (OSError, serial.SerialException):
        pass
    stop.set()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('serial_port')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=8888)
    a = ap.parse_args()
    while True:
        try:
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout = a.serial_port, a.baud, 0.01
            ser.dtr = ser.rts = False
            ser.open()
            sock = socket.create_connection((a.host, a.port), timeout=5)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # small micro-ROS packets out at once
            sock.settimeout(0.05)
        except (OSError, serial.SerialException) as e:
            print(f'waiting ({e})')
            time.sleep(2)
            continue
        print(f'bridging {a.serial_port} <-> {a.host}:{a.port}')

        def sock_read():
            try:
                data = sock.recv(4096)
            except socket.timeout:
                return b''
            if not data:
                raise OSError('tcp closed')
            return data

        stop = threading.Event()
        # Serial side: what is waiting, at once. read(4096) held every ESP message up to the 10 ms
        # timeout while the other direction went straight through, so the ESP's clock sync (which
        # assumes both ways take equally long) set its clock ahead of the host (2026-10-30).
        threads = [threading.Thread(target=pump, args=(lambda: ser.read(max(1, ser.in_waiting)), sock.sendall, stop), daemon=True),
                   threading.Thread(target=pump, args=(sock_read, ser.write, stop), daemon=True)]
        for t in threads:
            t.start()
        stop.wait()
        ser.close()
        sock.close()
        print('disconnected, retrying')
        time.sleep(1)


if __name__ == '__main__':
    main()
