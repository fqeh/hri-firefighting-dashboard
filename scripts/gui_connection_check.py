#!/usr/bin/env python3
"""Verify local noVNC HTTP and the WebSocket-to-real-VNC handshake.

This proves desktop transport, not scene visibility; inspect gui_screenshot.sh's
actual X11 image separately before claiming the Gazebo scene is visible.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
from urllib.request import urlopen


def read_exact(stream, count):
    result = b''
    while len(result) < count:
        chunk = stream.read(count - len(result))
        if not chunk:
            raise RuntimeError('Desktop transport closed early')
        result += chunk
    return result


def check(host, port):
    with urlopen(f'http://{host}:{port}/vnc.html', timeout=5) as response:
        html = response.read()
        if response.status != 200 or b'noVNC' not in html:
            raise RuntimeError('noVNC HTML unavailable')
    key = base64.b64encode(os.urandom(16)).decode()
    request = (f'GET /websockify HTTP/1.1\r\nHost: {host}:{port}\r\n'
               'Upgrade: websocket\r\nConnection: Upgrade\r\n'
               f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n'
               'Sec-WebSocket-Protocol: binary\r\n\r\n')
    with socket.create_connection((host, port), timeout=5) as client:
        client.sendall(request.encode())
        stream = client.makefile('rb')
        status = stream.readline().decode().strip()
        if ' 101 ' not in status:
            raise RuntimeError(f'WebSocket upgrade failed: {status}')
        headers = {}
        while True:
            line = stream.readline()
            if line == b'\r\n':
                break
            if not line:
                raise RuntimeError('Incomplete WebSocket headers')
            name, value = line.decode().split(':', 1)
            headers[name.strip().lower()] = value.strip()
        expected = base64.b64encode(hashlib.sha1(
            (key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
        if headers.get('sec-websocket-accept') != expected:
            raise RuntimeError('Incorrect WebSocket acceptance key')
        first, second = read_exact(stream, 2)
        if first & 15 != 2 or second & 128:
            raise RuntimeError('Expected an unmasked binary server frame')
        length = second & 127
        if length == 126:
            length = struct.unpack('!H', read_exact(stream, 2))[0]
        elif length == 127:
            length = struct.unpack('!Q', read_exact(stream, 8))[0]
        if length > 1024:
            raise RuntimeError('Unexpected initial desktop frame length')
        banner = read_exact(stream, length)
        if not banner.startswith(b'RFB 003.'):
            raise RuntimeError(f'Expected actual VNC protocol banner, received {banner!r}')
    return {'status':'passed', 'noVNC_http_status':200, 'websocket_status':101,
            'vnc_protocol_banner':banner.decode().strip(),
            'url':f'http://{host}:{port}/vnc.html?autoconnect=true&resize=scale',
            'scope':'desktop transport; scene visibility requires separate screenshot inspection'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=6080)
    parser.add_argument('--output', type=Path, default=Path('results/gui/connection.json'))
    args = parser.parse_args()
    try:
        result = check(args.host, args.port)
    except Exception as error:
        result = {'status':'failed', 'error':str(error)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    if result['status'] != 'passed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
