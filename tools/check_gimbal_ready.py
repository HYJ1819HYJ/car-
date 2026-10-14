"""Status-only readiness check before the dedicated tracking launch."""
import os
import re
import select
import sys
import termios
import time

device = '/dev/ttyUSB1'
try:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
except OSError as error:
    sys.exit('云台串口无法打开：{}'.format(error))
old = termios.tcgetattr(fd)
states = []
buffer = bytearray()
try:
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    deadline = time.monotonic() + 2.0
    next_query = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_query:
            if os.write(fd, bytes.fromhex('AB 03 AE')) != 3:
                raise RuntimeError('状态查询写入不完整')
            next_query = now + 0.25
        readable, _, _ = select.select([fd], [], [], 0.02)
        if not readable:
            continue
        try:
            buffer.extend(os.read(fd, 4096))
        except BlockingIOError:
            continue
        while b'\n' in buffer:
            line, _, rest = buffer.partition(b'\n')
            buffer[:] = rest
            text = line.decode('ascii', errors='replace').strip()
            if text.startswith('STAT,'):
                states.append({key: int(value) for key, value in
                               re.findall(r'([a-zA-Z_0-9]+)=(-?\d+)', text)})
finally:
    termios.tcsetattr(fd, termios.TCSANOW, old)
    os.close(fd)

if len(states) < 3:
    sys.exit('未收到足够的完整云台状态，未启动跟踪。')
print('最新云台状态：{}'.format(states[-1]), flush=True)
if not all(state.get('ready') == 1 and state.get('mode') == 0 for state in states[-3:]):
    sys.exit('云台未连续报告 ready=1、mode=0，未启动跟踪。请先核对舵机 ID、总线接线和实际固件。')
print('云台已就绪，可以启动摄像头目标跟踪。', flush=True)
