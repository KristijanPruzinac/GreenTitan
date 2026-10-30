#!/data/data/com.termux/files/usr/bin/python
"""Relay the ESP32's USB serial (OTG) to the micro-ROS agent's TCP socket, without root.

Android only lets an app that holds the user's USB permission open the device; Termux:API gets it
and hands this script an open file descriptor (termux-usb). libusb then drives the board's
USB-UART chip directly (CH340/CH341 or CP210x), so no kernel tty is needed. Same byte relay as
ros2_ws/src/sim_env/scripts/serial_tcp_bridge.py, so the agent side is the same socat pty.

    termux-usb -l                                   # e.g. ["/dev/bus/usb/001/002"]
    termux-usb -r -e ~/esp_usb_bridge.py /dev/bus/usb/001/002

Env: BRIDGE_HOST (127.0.0.1), BRIDGE_PORT (8888), BRIDGE_BAUD (921600).
Needs: pkg install python libusb; pip install libusb1.
"""
import os
import socket
import sys
import threading
import time

import usb1

HOST = os.environ.get('BRIDGE_HOST', '127.0.0.1')
PORT = int(os.environ.get('BRIDGE_PORT', '8888'))
BAUD = int(os.environ.get('BRIDGE_BAUD', '921600'))

CH34X = {(0x1a86, 0x7523), (0x1a86, 0x5523), (0x1a86, 0x7522)}
CP210X = {(0x10c4, 0xea60), (0x10c4, 0xea70)}


def ch34x_baud(baud):
    # usb-serial-for-android Ch34xSerialDriver.setBaudRate
    if baud == 921600:
        factor, divisor = 0xf300, 7
    else:
        factor, divisor = 1532620800 // baud, 3
        while factor > 0xfff0 and divisor > 0:
            factor >>= 3
            divisor -= 1
        factor, divisor = 0x10000 - factor, divisor | 0x80
    return (factor & 0xff00) | divisor, factor & 0xff


def init_ch34x(h):
    out = lambda req, val, idx: h.controlWrite(0x40, req, val, idx, b'', 1000)
    h.controlRead(0xc0, 0x5f, 0, 0, 2, 1000)                 # version
    out(0xa1, 0, 0)                                          # serial init
    out(0x9a, 0x2518, 0xc3)                                  # 8N1, rx+tx on
    out(0xa1, 0x501f, 0xd90a)
    v1, v2 = ch34x_baud(BAUD)
    out(0x9a, 0x1312, v1)
    out(0x9a, 0x0f2c, v2)
    out(0xa4, 0xffff, 0)                                     # DTR/RTS off: no ESP32 reset
    return 0x82, 0x02


def init_cp210x(h):
    out = lambda req, val, data=b'': h.controlWrite(0x41, req, val, 0, data, 1000)
    out(0x00, 0x0001)                                        # IFC_ENABLE
    out(0x1e, 0, BAUD.to_bytes(4, 'little'))                 # SET_BAUDRATE
    out(0x03, 0x0800)                                        # 8N1
    out(0x07, 0x0300)                                        # DTR/RTS low (mask set, values 0)
    return 0x81, 0x01


def main():
    fd = int(sys.argv[1] if len(sys.argv) > 1 else os.environ['TERMUX_USB_FD'])
    ctx = usb1.USBContext(with_device_discovery=False)
    h = ctx.wrapSysDevice(fd)
    dev = h.getDevice()
    vidpid = (dev.getVendorID(), dev.getProductID())
    if h.kernelDriverActive(0):
        h.detachKernelDriver(0)
    h.claimInterface(0)
    if vidpid in CH34X:
        ep_in, ep_out = init_ch34x(h)
    elif vidpid in CP210X:
        ep_in, ep_out = init_cp210x(h)
    else:
        sys.exit(f'unsupported USB-UART {vidpid[0]:04x}:{vidpid[1]:04x}')
    print(f'usb {vidpid[0]:04x}:{vidpid[1]:04x} at {BAUD} baud', flush=True)

    while True:
        try:
            sock = socket.create_connection((HOST, PORT), timeout=5)
        except OSError as e:
            print(f'waiting for agent socket ({e})', flush=True)
            time.sleep(2)
            continue
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.05)
        print(f'bridging usb <-> {HOST}:{PORT}', flush=True)
        stop = threading.Event()

        def usb_to_tcp():
            try:
                while not stop.is_set():
                    try:
                        data = h.bulkRead(ep_in, 4096, 10)
                    except usb1.USBErrorTimeout as e:
                        data = getattr(e, 'received', b'')
                    if data:
                        sock.sendall(data)
            except (OSError, usb1.USBError) as e:
                print(f'usb->tcp: {e}', flush=True)
            stop.set()

        def tcp_to_usb():
            try:
                while not stop.is_set():
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        raise OSError('tcp closed')
                    h.bulkWrite(ep_out, data, 1000)
            except (OSError, usb1.USBError) as e:
                print(f'tcp->usb: {e}', flush=True)
            stop.set()

        threads = [threading.Thread(target=f, daemon=True) for f in (usb_to_tcp, tcp_to_usb)]
        for t in threads:
            t.start()
        stop.wait()
        sock.close()
        for t in threads:
            t.join(1)
        time.sleep(1)


if __name__ == '__main__':
    main()
